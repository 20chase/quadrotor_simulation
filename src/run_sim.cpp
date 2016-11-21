#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>

#include "../include/QuadRotor.h"

using namespace std;

extern double attitudeTime;

ros::Publisher position_pub;

void rkSolver(double solverLength, double simulationTime);
void attitudeControl(double roll, double pitch, double yaw, double hight, double controlLength);
void positionControl(double desiredX, double desiredY, double desiredZ);


class Odometry{
public:
    float pos_x;
    float pos_y;
    float pos_z;
    float vel_x;
    float vel_y;
    float vel_z;
    float imu_x;
    float imu_y;
    float imu_z;
    float imu_w;

};

Odometry Odom;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "QuadRotor");
    ros::NodeHandle nh;
    ros::Rate loopRate(20);

    position_pub = nh.advertise<nav_msgs::Odometry>("QuadRotor/Position", 10);





	if (initialize() < 0)
	{
		cout << "Initialized Failed!" << endl;
		return -1;
	}
	else
	{
		cout << "Initialized Successful!" << endl;
	}

	cout << "This is a Simulation Project For Quadrotor! " << endl;


    while (true)
    {
        positionControl(1, -1, 1);
        positionControl(2, 0, 1.5);
        positionControl(1, 1, 1);
        positionControl(0, 0, 0.5);
        positionControl(-1, -1, 1);
        positionControl(-2, 0, 1.5);
        positionControl(-1, 1, 1);
        positionControl(0, 0, 0.5);

    }






    /*
    char writeFlag;
    cout << "if you want to write the trajectory(y/n): ";
    cin >> writeFlag;

    if (writeFlag == 'y')
    {
        writeTrajectory(trajectory, circleNum);
    }
    */

	return 0;
}

void rkSolver(double solverLength, double simulationTime)
{
    double circleFlag = 0;
    MatrixXd k1(3, 1),k2(3, 1),k3(3, 1),k4(3, 1);

    while (circleFlag < simulationTime)
    {
        k1 = computeOmegaDot(omega, computeTorque(w), w);
        k2 = computeOmegaDot(omega + solverLength * k1 /2, computeTorque(w), w);
        k3 = computeOmegaDot(omega + solverLength * k2 /2, computeTorque(w), w);
        k4 = computeOmegaDot(omega + solverLength * k3, computeTorque(w), w);

        omega = omega + solverLength * (k1 + 2 * k2 + 2* k3 + k4) / 6;
        angle = angle + omega * solverLength;

        acceleration = computeAcceleration(computeThrust(w), computeRotation(angle));
        velocity = velocity + acceleration * solverLength;
        position = position + velocity * solverLength;

        circleFlag += solverLength;
    }
}

void attitudeControl(double roll, double pitch, double yaw, double hight, double controlLength)
{

    double circleFlag = 0;
    int circleNum = 0;
    int arriveFlag = 0;
    int arriveNum = 0;

    MatrixXd controlAngle(3, 1), desiredAngle(3, 1);
    double controlH, desiredH;

    desiredAngle(0, 0) = roll;
    desiredAngle(1, 0) = pitch;
    desiredAngle(2, 0) = yaw;
    desiredH = hight;

    geometry_msgs::TransformStamped odomTransForm;
    odomTransForm.header.frame_id = "odom";
    odomTransForm.child_frame_id = "base_footprint";

    ros::Time currentTime;

    tf::TransformBroadcaster bf;

    while (!arriveFlag && circleFlag < 0.05)
    {
        rkSolver(0.001, attitudeTime);

        controlAngle = P4Attitude.cwiseProduct(desiredAngle - angle) + D4Attitude.cwiseProduct(-omega);
        controlH = P4Position(2, 0) * (desiredH - position(2, 0)) + D4Position(2, 0) * (-velocity(2, 0));

//        cout << "Time: " <<circleFlag <<"    position: \n" << position << endl;

        w(2, 0) = (controlH + G) * M / (4 * Kl) + controlAngle(2, 0) / (4 * Kd) + controlAngle(1, 0) / (2 * Kl * L);
        w(0, 0) = w(2, 0) - controlAngle(1, 0) / (2 * Kl * L);
        w(1, 0) = (controlH + G) * M / (4 * Kl) - controlAngle(2, 0) / (4 * Kd) + controlAngle(0, 0) / (2 * Kl * L);
        w(3, 0) = w(1, 0) - controlAngle(0, 0) / (2 * Kl * L);

        for (int i = 0; i < 4; i++)
        {
            if (w(i, 0) > 16000 * 16000)
            {
                w(i, 0) = 16000 * 16000;
            }

            if (w(i, 0) < 0)
            {
                w(i, 0) = 0;
            }
        }

        if (abs(controlAngle.sum()) < 0.00005 && abs(controlH) < 0.5)
        {
            cout << "controlH = " << controlH << endl;
            arriveNum++;
            if (arriveNum > 10)
            {
//                cout << "Arrived at Desired Attitude!" << endl;
                arriveFlag = 1;
            }
        }
        else
        {
            arriveNum = 0;
        }

        currentTime = ros::Time::now();

        geometry_msgs::Quaternion odomQuaternion;


        Odom.pos_y = float (position(1, 0));
        Odom.pos_x = float (position(0, 0));
        Odom.pos_z = float (position(2, 0));
        Odom.imu_x = float(sin(angle(0, 0) / 2) * cos(angle(1, 0) / 2) * cos(angle(2, 0) / 2) - cos(angle(0, 0) / 2) * sin(angle(1, 0) / 2) * sin(angle(2, 0) / 2));
        Odom.imu_y = float(cos(angle(0, 0) / 2) * sin(angle(1, 0) / 2) * cos(angle(2, 0) / 2) + sin(angle(0, 0) / 2) * cos(angle(1, 0) / 2) * sin(angle(2, 0) / 2));
        Odom.imu_z = float(cos(angle(0, 0) / 2) * cos(angle(1, 0) / 2) * sin(angle(2, 0) / 2) - sin(angle(0, 0) / 2) * sin(angle(1, 0) / 2) * cos(angle(2, 0) / 2));
        Odom.imu_w = float(cos(angle(0, 0) / 2) * cos(angle(1, 0) / 2) * cos(angle(2, 0) / 2) + sin(angle(0, 0) / 2) * sin(angle(1, 0) / 2) * sin(angle(2, 0) / 2));

        odomQuaternion.x = Odom.imu_x;
        odomQuaternion.y = Odom.imu_y;
        odomQuaternion.z = Odom.imu_z;
        odomQuaternion.w = Odom.imu_w;

        odomTransForm.header.stamp = currentTime;
        odomTransForm.transform.translation.x = Odom.pos_x;
        odomTransForm.transform.translation.y = Odom.pos_y;
        odomTransForm.transform.translation.z = Odom.pos_z;
        odomTransForm.transform.rotation.x = Odom.imu_x;
        odomTransForm.transform.rotation.y = Odom.imu_y;
        odomTransForm.transform.rotation.z = Odom.imu_z;
        odomTransForm.transform.rotation.w = Odom.imu_w;

        nav_msgs::Odometry odom;
        nav_msgs::Path     path;
        geometry_msgs::PoseStamped pose;

        odom.header.stamp = currentTime;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        path.header.stamp   = currentTime;
        path.header.frame_id = "odom";

        odom.pose.pose.position.x = Odom.pos_x;
        odom.pose.pose.position.y = Odom.pos_y;
        odom.pose.pose.position.z = Odom.pos_z;
        odom.pose.pose.orientation = odomQuaternion;


        bf.sendTransform(odomTransForm);
        position_pub.publish(odom);

        ros::spinOnce();

        usleep(controlLength * 1000 * 1000);

        circleNum++;
        circleFlag += controlLength;
    }
}

void positionControl(double desiredX, double desiredY, double desiredZ)
{
    int arriveNum = 0;
    while (true)
    {
        MatrixXd controlPosition(3, 1), desiredPosition(3, 1);

        desiredPosition(0, 0) = desiredX;
        desiredPosition(1, 0) = desiredY;
        desiredPosition(2, 0) = desiredZ;

        controlPosition = P4Position.cwiseProduct(desiredPosition - position) + D4Position.cwiseProduct(- velocity);

        controlPosition(0, 0) = limitMax(controlPosition(0, 0), 0.628);
        controlPosition(1, 0) = limitMax(controlPosition(1, 0), 0.628);
        controlPosition(2, 0) = limitMax(controlPosition(2, 0), 0.628);

        cout << "position: \n" << position << endl;
        cout << "controlPosition: \n" << controlPosition << endl;


        attitudeControl(-controlPosition(1, 0), controlPosition(0, 0), 0, desiredZ, attitudeTime);

        if (abs(controlPosition(0, 0)) < 0.001 && abs(controlPosition(1, 0)) < 0.001)
        {
            arriveNum ++;
            if (arriveNum > 2)
            {
                attitudeControl(0, 0, 0, desiredZ, attitudeTime);
                break;
            }
        }
        else
        {
            arriveNum = 0;
        }

    }
}

