/*
 This ROS node is meant to sit between the controller and the robot to intercept commands and stop any from being executed that place the robot on a path towards imminent collision.
 */

#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

ros::Publisher pub_twist, pub_cmd, pub_cmd_cov, pub_future_pose;
float projection_time, cost_threshold, certainty, robot_radius;
int num_steps;
bool only_test_last;
std::string map_frame, base_link_frame;
tf::TransformListener* tf_listener;
nav_msgs::OccupancyGrid current_costmap;
nav_msgs::Odometry current_odometry;
int future_pose_seq = 0;
bool have_costmap = false;
bool have_odometry = false;

void publish_twist_cov(const geometry_msgs::TwistStamped stamped_twist){
	geometry_msgs::TwistWithCovarianceStamped out_twist_cov;
	out_twist_cov.header = stamped_twist.header;
	out_twist_cov.twist.twist = stamped_twist.twist;
	pub_cmd_cov.publish(out_twist_cov);
}

void received_cmd(const geometry_msgs::TwistStamped in_cmd){
    // Check if command has movement
    if(in_cmd.twist.linear.x == 0 && in_cmd.twist.angular.z == 0){
        return;
    }
    if(have_odometry == false || have_costmap == false){
        ROS_INFO("Need odometry and costmap to calculate future pose.");
        return;
    }
    bool bad_cmd = false;
    geometry_msgs::TwistStamped out_cmd = in_cmd;
    // Get current pose
    geometry_msgs::PoseStamped future_pose;
    future_pose.header.seq = ++future_pose_seq;
    future_pose.header.stamp = out_cmd.header.stamp;
    // in base_link frame
    future_pose.header.frame_id = base_link_frame;
    future_pose.pose.position.x = 0;
    future_pose.pose.position.y = 0;
    future_pose.pose.position.z = 0;
    future_pose.pose.orientation.x = 0;
    future_pose.pose.orientation.y = 0;
    future_pose.pose.orientation.z = 0;
    future_pose.pose.orientation.w = 1;
    // Loop for num_steps
    float time_step = projection_time / float(num_steps);
    for(int t=0; t<num_steps; t++){
        // Transform quaternion to euler angles
        tf::Quaternion quat(future_pose.pose.orientation.x, future_pose.pose.orientation.y, future_pose.pose.orientation.z, future_pose.pose.orientation.w);
        tf::Matrix3x3 quat_mat(quat);
        double roll, pitch, yaw;
        quat_mat.getRPY(roll, pitch, yaw);
        // Vector projection using euler angles
        double ux = out_cmd.twist.linear.x * (cos(yaw)*cos(pitch))
                    + out_cmd.twist.linear.y * (cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll))
                    + out_cmd.twist.linear.z * (cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll));
        double uy = out_cmd.twist.linear.x * (sin(yaw)*cos(pitch))
                    + out_cmd.twist.linear.y * (sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll))
                    + out_cmd.twist.linear.z * (sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll));
        double uz = out_cmd.twist.linear.x * (-sin(pitch))
                    + out_cmd.twist.linear.y * (cos(pitch)*sin(roll))
                    + out_cmd.twist.linear.z * (cos(pitch)*cos(roll));
        // Calculate future pose
        future_pose.pose.position.x = future_pose.pose.position.x + time_step * ux;
        future_pose.pose.position.y = future_pose.pose.position.y + time_step * uy;
        future_pose.pose.position.z = future_pose.pose.position.z + time_step * uz;
        roll = roll + time_step * out_cmd.twist.angular.x;
        pitch = pitch + time_step * out_cmd.twist.angular.y;
        yaw = yaw + time_step * out_cmd.twist.angular.z;
        quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
        future_pose.pose.orientation.x = quat.getX();
        future_pose.pose.orientation.y = quat.getY();
        future_pose.pose.orientation.z = quat.getZ();
        future_pose.pose.orientation.w = quat.getW();
        if((!only_test_last) || (t==num_steps-1)){
            // Find pose on costmap
            tf::StampedTransform transform;
            try{
                tf_listener->lookupTransform(current_costmap.header.frame_id, future_pose.header.frame_id, ros::Time(0), transform);
            } catch(tf::TransformException exception){
                ROS_ERROR("%s", exception.what());
                bad_cmd = true;
                return;
            }
            quat = transform.getRotation();
            tf::Matrix3x3 quat_mat2(quat);
            quat_mat2.getRPY(roll, pitch, yaw);
            double costmap_pose_x = cos(yaw) * future_pose.pose.position.x + transform.getOrigin().getX() - current_costmap.info.origin.position.x;
            double costmap_pose_y = sin(yaw) * future_pose.pose.position.y + transform.getOrigin().getY() - current_costmap.info.origin.position.y;
            int pc_x = int(costmap_pose_x / current_costmap.info.resolution);
            int pc_y = int(costmap_pose_y / current_costmap.info.resolution);
            // Look at bounding box on costmap for collisions. This is a box around the circle circumscribing the robot (defined by robot_radius).
            int shift = int(robot_radius / current_costmap.info.resolution);
            for(int i=-shift; i<=shift; i++){
                for(int j=-shift; j<=shift; j++){
                    if((current_costmap.info.width * (pc_y + i) + (pc_x + j)) > (current_costmap.info.width * current_costmap.info.height)){
                        ROS_INFO("Pose estimate is not on the cost map!");
                        bad_cmd = true;
                        break;
                    }
                    if(current_costmap.data[current_costmap.info.width * (pc_y + i) + (pc_x + j)] >= cost_threshold){
                        ROS_INFO("Future collision detected! Sending zero command.");
                        bad_cmd = true;
                        break;
                    }
                }
            }
        }
    }
    // Zero out cmd if needed
    if(bad_cmd){
        out_cmd.twist.linear.x = 0.0;
        out_cmd.twist.linear.y = 0.0;
        out_cmd.twist.linear.z = 0.0;
        out_cmd.twist.angular.x = 0.0;
        out_cmd.twist.angular.y = 0.0;
        out_cmd.twist.angular.z = 0.0;
    }
    // Publish and return
    pub_future_pose.publish(future_pose);
    pub_twist.publish(out_cmd.twist);
    pub_cmd.publish(out_cmd);
    publish_twist_cov(out_cmd);
    return;
}

void received_twist(const geometry_msgs::Twist in_twist){
    geometry_msgs::TwistStamped stamped_twist;
    stamped_twist.header.seq = 0;
    stamped_twist.header.stamp = ros::Time::now();
    stamped_twist.header.frame_id = "";
    stamped_twist.twist = in_twist;
    received_cmd(stamped_twist);
}

void received_twist_bypass(const geometry_msgs::Twist in_twist){
    geometry_msgs::TwistStamped stamped_twist;
    stamped_twist.header.seq = 0;
    stamped_twist.header.stamp = ros::Time::now();
    stamped_twist.header.frame_id = "";
    stamped_twist.twist = in_twist;
    pub_cmd.publish(stamped_twist);
    publish_twist_cov(stamped_twist);
}

void received_costmap(const nav_msgs::OccupancyGrid in_costmap){
    current_costmap = in_costmap;
    have_costmap = true;
    //ROS_INFO("Costmap recieved.");
}

void received_odometry_filtered(const nav_msgs::Odometry in_odometry_filtered){
    current_odometry = in_odometry_filtered;
    have_odometry = true;
}

int main(int argc, char **argv){
    // Init
    ros::init(argc, argv, "imminent_collision_interceptor");
    ros::NodeHandle nh;
    ros::NodeHandle node_private("~");
    // Sub/Pub
    ros::Subscriber sub_twist = nh.subscribe("cmd_vel/twist", 1000, &received_twist);
    ros::Subscriber sub_cmd = nh.subscribe("cmd_vel/twist_stamped", 1000, &received_cmd);
    ros::Subscriber sub_costmap = nh.subscribe("local_costmap/costmap/costmap", 1000, &received_costmap);
    ros::Subscriber sub_odometry_filtered = nh.subscribe("odometry/filtered", 1000, &received_odometry_filtered);
    ros::Subscriber sub_twist_bypass = nh.subscribe("cmd_vel/bypass", 1000, &received_twist_bypass);
    pub_twist = nh.advertise<geometry_msgs::Twist>("cmd_vel/intercepted", 1000);
    pub_cmd = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel/intercepted_stamped", 1000);
    pub_cmd_cov = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("cmd_vel/intercepted_cov", 1000);
    pub_future_pose = nh.advertise<geometry_msgs::PoseStamped>("/future_pose", 1000);
    // TF and costmap
    tf_listener = new tf::TransformListener();
    current_costmap.header.stamp = ros::Time::now();
    // Get Parameters
    node_private.param<float>("projection_time", projection_time, 1.0);
    node_private.param<int>("num_steps", num_steps, 10);
    node_private.param<float>("cost_threshold", cost_threshold, 50.0);
    node_private.param<float>("certainty", certainty, 0.95);
    node_private.param<float>("robot_radius", robot_radius, 0.25);
    node_private.param<bool>("only_test_last", only_test_last, false);
    node_private.param<std::string>("map_frame", map_frame, "map");
    node_private.param<std::string>("base_link_frame", base_link_frame, "base_link");
    ros::spin();
}
