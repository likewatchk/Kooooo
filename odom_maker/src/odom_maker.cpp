/*This is the node that subscribes motor's pose and makes TF and Odom info*/

#include <ros/ros.h>
#include <algorithm>
#include <thread>

// INPUT [Motor position]
#include "odom_maker/motorPose.h"
#include <geometry_msgs/TwistStamped.h>

// OUTPUT [TF / Odometry]
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

#define pose2rad_const          2*3.141592653589 / 56500
#define R                       0.045

class Odom_maker{
public:
    Odom_maker() : seq(0), nh(), x(0.0), y(0.0), theta(0.0), left_pos_last(0), right_pos_last(0){
        last_br = ros::Time::now();
        last_cb = ros::Time::now();
        
        if (!nh.getParam("/mobile_robot_odometry/base_link_id", base_link_id)) 
            throw std::runtime_error("set base_link_id");
        if (!nh.getParam("/mobile_robot_odometry/odom_link_id", odom_link_id)) 
            throw std::runtime_error("set odom_link_id");
        if (!nh.getParam("/mobile_robot_odometry/separation_length", seperation_length)) 
            throw std::runtime_error("set seperation_length");
 
        motorPose_sub = nh.subscribe("/motorPose", 100, &Odom_maker::motorPoseCB, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 200);

        std::thread([&](){
            ros::Rate r(50);
            while(ros::ok()){
                broadcastTransform();
                r.sleep();
            }
        }).detach();
    }
    
    void motorPoseCB(const odom_maker::motorPoseConstPtr& ptr){
        //dt
        ros::Time cur = ros::Time::now();
        double dt = (cur - last_cb).toSec();

    //How to initialize last_pos. -> 0
    //What side is minus ?  ->  left
    //what number is for one-cycle  56500

        //cur_pose
        int left_pos_cur, right_pos_cur;
        left_pos_cur  = -(ptr->left_pos);
        right_pos_cur = ptr->right_pos;

        //diff_pose
        int diff_left_pos, diff_right_pos;
        diff_left_pos  = left_pos_cur  -  left_pos_last;
        diff_right_pos = right_pos_cur -  right_pos_last;

        //diff converted to rad
        double diff_left_rad, diff_right_rad;
        diff_left_rad  = pose2rad_const * diff_left_pos;
        diff_right_rad = pose2rad_const * diff_right_pos;

        //calc each w
        double w_left  = diff_left_rad  / dt;
        double w_right = diff_right_rad / dt;
       
        //calc each v
        left_wheel_velocity = R * w_left;
        right_wheel_velocity = R * w_right;

        last_cb = cur;
        // //calc V, W
        // double V = (R/2) * (w_right + w_left);
        // double W = (R/seperation_length) * (w_right - w_left);                
    }

    void broadcastTransform(){
        ros::Time cur = ros::Time::now();

        //Odom!
        double v = (left_wheel_velocity + right_wheel_velocity) / 2;
        double x_dot = v * std::cos(theta);
        double y_dot = v * std::sin(theta);
        double theta_dot = (right_wheel_velocity - left_wheel_velocity) / seperation_length;

        double dt = (cur - last_br).toSec();
        double dx = x_dot * dt;
        double dy = y_dot * dt;
        double dtheta = theta_dot * dt;

        x += dx;
        y += dy;
        theta += dtheta;

        //check Nan
        if(x != x) x = 0;
        if(y != y) y = 0;
        if(theta != theta) theta = 0;

        //generate tf msg
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, theta);
        transform.setRotation(q);

        TF_br.sendTransform(tf::StampedTransform(transform, cur, odom_link_id, base_link_id));

        //pub odom
        nav_msgs::Odometry odom;
        odom.header.seq = seq++;
        odom.header.stamp = cur;
        odom.header.frame_id = odom_link_id;
        odom.child_frame_id = base_link_id;

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        odom.twist.twist.linear.x = x_dot;
        odom.twist.twist.linear.y = y_dot;
        odom.twist.twist.angular.z = theta_dot;

        odom_pub.publish(odom);
        last_br = cur;
    }

private: //parameter members
    std::string base_link_id; //
    std::string odom_link_id; //
    double seperation_length; //
private: //Odometry fields
    int left_pos_last;  //
    int right_pos_last; //
    double x, y, theta; //
    ros::Time last_br; //
    ros::Time last_cb; //
    uint seq; //   
private: //shared variable
    double left_wheel_velocity;
    double right_wheel_velocity;
private: //ros dependency
    ros::Publisher odom_pub;
    ros::Subscriber motorPose_sub;
    ros::NodeHandle nh;
    tf::TransformBroadcaster TF_br;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Odom_maker");

    Odom_maker om;

    ros::spin();
    return 0;
}