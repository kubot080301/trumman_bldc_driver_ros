#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

#include <math.h>
#include <string>
#include <stdlib.h>
#include <iostream>

#include <serial/serial.h> 

#include <motor_driver/modbus.h>

serial::Serial ser; 
bool open_port();

namespace EVmotor
{
	int ROTATING_SPEED_UPPER_LIMIT = 3000;
	double AXIS_LENGTH = 0.55;
	double WHEEL_DIAMETER = 0.15;
	double REDUCTION_RATIO = 20;

	double WHEEL_SERVO_ROTATING_SPEED_RATIO = 60 * REDUCTION_RATIO / (WHEEL_DIAMETER * M_PI);
	double WHEEL_PERIMETER = WHEEL_DIAMETER * M_PI;

	int SERVO_STEP_MAX = 327670000;
	int SERVO_STEP_MIN = -327680000;

	// double max_speed = 0.4;
    double max_speed = 0.6; 

	struct Wheel_data
	{
		double LinearVelocity;
		double AngularVelocity;
		double timeTotalSeconds;
		double MoveDistanceL;
		double MoveDistanceR;
	}Wheel_read;
		
	uint8_t read_step_R[4] = {0};
	uint8_t read_step_L[4] = {0}; 

	ros::Time before;
	ros::Time now;

	double Yaw = 0;
	double PositionX = 0;
	double PositionY = 0;

	geometry_msgs::Pose pose;
	geometry_msgs::Twist twist;
	geometry_msgs::Quaternion qua;
	nav_msgs::Odometry Odometry;

	void ToQuaternion(double pitch, double roll, double yaw);
	void Odometer();
	int read_step();
	
	int control(uint8_t left,uint8_t right,int speedL,int speedR);
	void unit_speed(double linear_speed,double angular_speed);

	int SetPositionMode();
	int SetSpeedMode();

	int stop();
	int reset();
	int meesge();


}

bool open_port()
{
    try
    {
        ser.setPort("/dev/RS485");
        ser.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.setParity(serial::parity_none);
        ser.setStopbits(serial::stopbits_one);  
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return false;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return false;
    }
}

bool close_port()
{
    ser.flush();
    ser.close();
}

void EVmotor::ToQuaternion(double pitch, double roll, double yaw)
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    qua.w = (float)(cy * cr * cp + sy * sr * sp);
    qua.x = (float)(cy * sr * cp - sy * cr * sp);
    qua.y = (float)(cy * cr * sp + sy * sr * cp);
    qua.z = (float)(sy * cr * cp - cy * sr * sp);
}

void EVmotor::Odometer()
{
    double RightMoveSpeed = -(Wheel_read.MoveDistanceR / Wheel_read.timeTotalSeconds);
    double LeftMoveSpeed = Wheel_read.MoveDistanceL / Wheel_read.timeTotalSeconds;
    Wheel_read.LinearVelocity = (LeftMoveSpeed + RightMoveSpeed) / 2;
    Wheel_read.AngularVelocity = (RightMoveSpeed - LeftMoveSpeed) / AXIS_LENGTH;

    //printf("V = %lf W = %lf \n",Wheel_read.LinearVelocity,Wheel_read.AngularVelocity);

    double delta_x = Wheel_read.LinearVelocity * cos(Yaw) * Wheel_read.timeTotalSeconds;
    double delta_y = Wheel_read.LinearVelocity * sin(Yaw) * Wheel_read.timeTotalSeconds;
    double delta_th = Wheel_read.AngularVelocity * Wheel_read.timeTotalSeconds;
    // printf("R = %lf L = %lf line = %lf a = %lf x = %lf y = %lf z = %lf\n",RightMoveSpeed,LeftMoveSpeed,Wheel_read.LinearVelocity,Wheel_read.AngularVelocity ,delta_x,delta_y,delta_th);
    
    PositionX += delta_x;
    PositionY += delta_y;
    Yaw += delta_th;

    pose.position.x = (float)PositionX;
    pose.position.y = (float)PositionY;

    ToQuaternion(0, 0, Yaw);
    pose.orientation = qua;

    twist.linear.x = (float)Wheel_read.LinearVelocity;
    twist.linear.y = 0;
    twist.angular.z = (float)Wheel_read.AngularVelocity;

    Odometry.pose.pose = pose;
    Odometry.twist.twist = twist;
    Odometry.header.frame_id = "odom";
}

int EVmotor::read_step()
{
    int i;  
    uint8_t rsp[256];    

    uint8_t raw_req[] = { 0x00, 0x65, 0x02, 0x01, 0x63, 0x00, 0x00, 0x00, 0x00,0x02, 0x63, 0x00, 0x00, 0x00, 0x00};
    add_CRC(raw_req,sizeof(raw_req));
    ser.write(raw_req,sizeof(raw_req)+2);
    now = ros::Time::now();

    usleep(50000);

    int n = ser.available();    
    if(n)
    {
        ser.read(rsp,n);
        // for(i = 0; i<n;i++)
        //    printf("[%X] ",rsp[i]);
        // printf("\n");
    }

    uint32_t now_step,before_step,step_dis;
    //Left Step////////////////////////////////////////
    if(rsp[0]==0x01 && rsp[1]==0x66)
    {
    	now_step = ((uint32_t)rsp[2] << 24) | ((uint32_t)rsp[3] << 16) | ((uint16_t)rsp[4]) << 8 | rsp[5];
    	// printf("now = %d\n",(int)now_step );
    }
    before_step = ((uint32_t)read_step_L[0] << 24) | ((uint32_t)read_step_L[1] << 16) | ((uint16_t)read_step_L[2]) << 8 | read_step_L[3];
    
    step_dis = now_step - before_step;
    if(rsp[2]==0xFF && read_step_L[0]==0x00)
    	step_dis = step_dis - 0xFFFFFFFF;
    else if(rsp[2]==0x00 && read_step_L[0]==0xFF)
    	step_dis = step_dis + 0xFFFFFFFF;

    for(i = 0;i < 4; i++ )
        read_step_L[i] = rsp[i+2]; 
    // printf("step_dis = %d\n",(int)step_dis );
    Wheel_read.MoveDistanceL = (int)step_dis / 10000.0 / REDUCTION_RATIO * WHEEL_PERIMETER;

    //Right Step///////////////////////////////////////
    if(rsp[8]==0x02 && rsp[9]==0x66)
    {
    	now_step = ((uint32_t)rsp[10] << 24) | ((uint32_t)rsp[11] << 16) | ((uint16_t)rsp[12]) << 8 | rsp[13];
    	// printf("now = %d\n",(int)now_step );
    }
    before_step = ((uint32_t)read_step_R[0] << 24) | ((uint32_t)read_step_R[1] << 16) | ((uint16_t)read_step_R[2]) << 8 | read_step_R[3];
    
    step_dis = now_step - before_step;
    if(rsp[10]==0xFF && read_step_R[0]==0x00)
    	step_dis = step_dis - 0xFFFFFFFF;
    else if(rsp[10]==0x00 && read_step_R[0]==0xFF)
    	step_dis = step_dis + 0xFFFFFFFF;

    for(i = 0;i < 4; i++ )
        read_step_R[i] = rsp[i+10]; 
    // printf("step_dis = %d\n",(int)step_dis );
    Wheel_read.MoveDistanceR = (int)step_dis / 10000.0 / REDUCTION_RATIO * WHEEL_PERIMETER;

    //total time////////////////////////////////
    Wheel_read.timeTotalSeconds = (now.sec + (double)now.nsec/1000000000) - (before.sec + (double)before.nsec/1000000000);
    before = now;
    Odometer();
}

int EVmotor::stop()
{
    int i;  
    uint8_t rsp[256];    

    uint8_t raw_req[] = {0x00, 0x65, 0x02, 0x01, 0x68, 0x00, 0x00, 0x00, 0x00, 0x02, 0x68, 0x00, 0x00, 0x00, 0x00};
    add_CRC(raw_req,sizeof(raw_req));
    ser.write(raw_req,sizeof(raw_req)+2);

    usleep(50000);

}

int EVmotor::control(uint8_t left,uint8_t right,int speedL,int speedR)
{
	int i;  
    uint8_t rsp[256];    

    int speedL_P = speedL;
    int speedR_P = speedR;

    // if(left == right && speedL_P > 400)
    // 	speedL_P = 400;
    // else if (speedL_P > 800)
    //     speedL_P = 800;

    // if(left == right && speedR_P > 400)
    //     speedR_P = 400;
    // else if (speedR_P > 800)
    //     speedR_P = 800;

    double over = max_speed * WHEEL_SERVO_ROTATING_SPEED_RATIO;

    if(speedL_P > over)
        speedL_P = over;
    if(speedR_P > over)
        speedR_P = over;

    uint8_t speedL_High = (uint8_t)(speedL_P/256);
    uint8_t speedL_Low = (uint8_t)(speedL_P%256);
    uint8_t speedR_High = (uint8_t)(speedR_P/256);
    uint8_t speedR_Low = (uint8_t)(speedR_P%256);

    uint8_t raw_req[] = {0x00, 0x65, 0x02, 0x01,left, 0x00, 0x00,speedL_High,speedL_Low, 0x02, right, 0x00, 0x00,speedR_High,speedR_Low};
    add_CRC(raw_req,sizeof(raw_req));
    ser.write(raw_req,sizeof(raw_req)+2);

    usleep(50000);
	
}

void EVmotor::unit_speed(double linear_speed,double angular_speed)
{
	double wl2 = AXIS_LENGTH * angular_speed / 2;

    int speed_L =  (linear_speed - wl2) * WHEEL_SERVO_ROTATING_SPEED_RATIO;
    int speed_R =  (linear_speed + wl2) * WHEEL_SERVO_ROTATING_SPEED_RATIO;

	uint8_t left;
    uint8_t right;

    if(speed_R >= 0)
        right = 0x66;
    else if(speed_R  <0)
        right = 0x65;
    else
        control(0x65,0x66,0,0);
    

    if(speed_L >= 0)
        left = 0x65;
    else if(speed_L < 0)
        left = 0x66;
    else
        control(0x65,0x66,0,0);

	control(left,right, abs(speed_L) , abs(speed_R) );
  	// printf("L = %d\n", abs(speed_L) );
 	// printf("R = %d\n", abs(speed_R) );
	

}

int EVmotor::SetPositionMode()
{
	int i;  
    uint8_t rsp[256];  

    uint8_t raw_req[] = {0x01,0x06,0x08,0x00,0x00,0x02,0x0A,0x6B};
    ser.write(raw_req,sizeof(raw_req));

    usleep(50000);

    int n = ser.available();

    if(n)
    {
        ser.read(rsp,n);
        for(i = 0; i<n;i++)
          printf("[%X] ",rsp[i]);
        printf("\n");

    }

    uint8_t raw_req2[] = {0x02,0x06,0x08,0x00,0x00,0x02,0x0A,0x58};
    ser.write(raw_req2,sizeof(raw_req2));

    usleep(50000);

    n = ser.available();

    if(n)
    {
        ser.read(rsp,n);
        for(i = 0; i<n;i++)
          printf("[%X] ",rsp[i]);
        printf("\n");

    }
}

int EVmotor::SetSpeedMode()
{
	int i;  
    uint8_t rsp[256];  

    uint8_t raw_req[] = {0x01 ,0x06 ,0x08 ,0x00 ,0x00, 0x00, 0x8B, 0xAA };
    ser.write(raw_req,sizeof(raw_req));

    usleep(50000);

   	int n = ser.available();

    if(n)
    {
        ser.read(rsp,n);
        for(i = 0; i<n;i++)
          printf("[%X] ",rsp[i]);
        printf("\n");

    }

    uint8_t raw_req2[] = {0x02, 0x06, 0x08, 0x00, 0x00, 0x00, 0x8B, 0x99 };
    ser.write(raw_req2,sizeof(raw_req2));

    usleep(50000);

    n = ser.available();

    if(n)
    {
        ser.read(rsp,n);
        for(i = 0; i<n;i++)
          printf("[%X] ",rsp[i]);
        printf("\n");

    }
}

int EVmotor::reset()
{
    int i;  
    uint8_t rsp[256];    

    uint8_t raw_req[] = { 0x00, 0x65, 0x02, 0x01, 0x72, 0x00, 0x00, 0x00, 0x00,0x02, 0x72, 0x00, 0x00, 0x00, 0x00};
    add_CRC(raw_req,sizeof(raw_req));
    ser.write(raw_req,sizeof(raw_req)+2);
    now = ros::Time::now();

    usleep(50000);

    int n = ser.available();    
    if(n)
    {
        ser.read(rsp,n);
        for(i = 0; i<n;i++)
           printf("[%X] ",rsp[i]);
        printf("\n");
    }
}

int EVmotor::meesge()
{
	//03 read 06 write
	int i;  
    uint8_t rsp[256];  

    uint8_t raw_req[] = {0x01 ,0x03 ,0x00 ,0x05,0x00, 0x03,0x15,0xCA};
    add_CRC(raw_req,sizeof(raw_req));
    ser.write(raw_req,sizeof(raw_req));

    usleep(50000);

   	int n = ser.available();

    if(n)
    {
        ser.read(rsp,n);
        for(i = 0; i<n;i++)
          printf("[%X] ",rsp[i]);
        printf("\n");

    }

}
