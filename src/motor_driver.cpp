#include <motor_driver/EVmotor.h>

double v = 0, w = 0;

void sub_speed(const geometry_msgs::Twist::ConstPtr& msg)
{
    v = msg->linear.x;
    w = msg->angular.z;
    EVmotor::unit_speed(v,w);
}

int main (int argc, char** argv)
{

    ros::init(argc, argv, "motor_driver");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    ros::Subscriber power_sub = nh.subscribe("cmd_vel", 1, sub_speed);
    ros::Publisher odom = nh.advertise<nav_msgs::Odometry>("odom", 1);

    nh.getParam("max_speed",EVmotor::max_speed);    
    
    EVmotor::before = ros::Time::now();

    // SetPositionMode();

    open_port();

    while(ros::ok())
    {
        ros::spinOnce();
        EVmotor::read_step();    
        odom.publish(EVmotor::Odometry);

	    loop_rate.sleep();
    }

    close_port();
    
    return 0;
}
