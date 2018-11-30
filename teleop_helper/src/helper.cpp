#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/TwistStamped.h>

class helper{
public:
    helper() : nh(), seq(0) {
        pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 10);
        sub = nh.subscribe<geometry_msgs::Twist>("/inter_cmd", 100, &helper::helpCB, this);
    }
    void helpCB(const geometry_msgs::TwistConstPtr& ptr){
        geometry_msgs::TwistStamped vel;
        vel.header.seq = seq++;
        vel.header.stamp = ros::Time::now();
        vel.header.frame_id ="map";
        vel.twist = *ptr;
        pub.publish(vel);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    int seq;
};

int main(int argc, char* argv[]){
    ros::init(argc, argv, "teleop_helper");
    
    helper h;

    ros::spin();
    return 0;
}
