#include "rosThread.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <unistd.h>

using namespace std;

RosThread::RosThread()
{

    // Intialize your global variables here

    robotX = 0;
    robotY = 0;
    robotTh = 0;

    targetX = 0;
    targetY = 0;

    travelDistance = 0;
}

RosThread::~RosThread()
{
}

void RosThread::work(){

    velPub = n.advertise<minik_ros_wrapper::minikSetVelocityMsg>("minik_ros_wrapper/minikSetVelocityMsg", 1);
    odomSub = n.subscribe("odom", 1, &RosThread::odomHandler, this);
    targetSub = n.subscribe("target", 1, &RosThread::targetHandler, this);
    obstacleSub = n.subscribe("obstacles", 1, &RosThread::obstacleHandler, this);

    ros::Rate loop(30);
    while(ros::ok())
    {
        // Write your algorithm here
	//_________________________________________	
	double k=5;
	double dxU[2]=no_obstacle_gradient(robotX,robotY,targetX,targetY,k);
	int lr=1;
	double pi=3.14;
	dxU[1]=dxU[1]*lr;
	dxU[2]=dxU[2]*lr;
	double Vmax,vx=0,vy=0,v=0;
	Vmax=0.3;
	double b=0.3; //tekerler arasý mesafe
	norm=euclidian_distance(robotX,robotY,0,0)	
	if (norm > 0.000000001){
	
        vx = -Vmax*(dxU[1]/norm);
        vy = -Vmax*(dxU[2]/norm);
    }
    else{
        vx = -dxU[1];
        vy = -dxU[2];
    }
    end
	double theta_des=atan2(-vx,vy);
	v=vy*cos(theta_des)-vx*sin(theta_des);
	w=(theta_des-robotTh)*180/pi;
	
	if (w < -180){
        w = w + 360
    }
    if (w > 180){
        w = w - 360
	}
	w=(-1*w);
	
	double vr=v+(3*w)/20;
	double vl=v-(3*w)/20;
	sendVelocityCommand(vl,-vr);
	
		
	//________________________________________
        demoLoop();

        ros::spinOnce();
        loop.sleep();
    }

    qDebug() << "Quitting";
    ros::shutdown();
}

void RosThread::demoLoop()
{
    //sendVelocityCommand(0.3,-0.3);
    cout << "X: " << robotX << " \t Y: " << robotY << " \t Theta: " << robotTh << endl;
    if (obstacles.size() > 0){
        cout << "oX: " << obstacles[0].x << " \t oY: " << obstacles[0].y << " \t oTheta: " << obstacles[0].r << endl;
    }
}


void RosThread::odomHandler(const nav_msgs::OdometryConstPtr &odomMsg){

    //  ^ Y
    //  |				--> Theta
    //  |			   |
    //   -----> X   	--<
    robotX = odomMsg->pose.pose.position.x;
    robotY = odomMsg->pose.pose.position.y;

    tf::Quaternion q(odomMsg->pose.pose.orientation.x, odomMsg->pose.pose.orientation.y,
                     odomMsg->pose.pose.orientation.z, odomMsg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    robotTh = yaw;

    travelDistance += sqrt(pow(robotX - _lastX, 2) + pow(robotY - _lastY, 2));
    _lastX = robotX;
    _lastY = robotY;
}

void RosThread::targetHandler(const geometry_msgs::PoseConstPtr &targetMsg){

    targetX = targetMsg->position.x;
    targetY = targetMsg->position.y;

}

void RosThread::obstacleHandler(const std_msgs::Float32MultiArrayConstPtr &obstacleMsg){

    obstacles.clear();

    for (int i=0; i<obstacleMsg->layout.dim[0].size; i++){

        Obstacle temp;

        temp.x = obstacleMsg->data[i*3];
        temp.y = obstacleMsg->data[i*3+1];
        temp.r = obstacleMsg->data[i*3+2];

        obstacles.push_back(temp);
    }

}

double euclidian_distance(double xR,double yR,double xT,double yT){
	double sum=0;
		sum=sqrt(pow((xT-xR),2)+pow((yT-yR),2))
		return sum;
}

double compute_gamma(double xR,double yR,double xT,double yT){
	double gamma =0;
	gamma=pow((xT-xR),2)+pow((yT-yR),2);
	return gamma;
}

double no_obstacle_gradient(double xR,double yR,double xT,double yT,double k){ // xR,yR represents robot position xT,yT represents target position
	gamma=compute_gamma(xR,yR,xT,yT);
	double grad[2];
	grad[1]=(2*k*(gamma^(k-1))*(xR-xT));
	grad[2]=(2*k*(gamma^(k-1))*(yR-yT));
	return grad;
}

void RosThread::sendVelocityCommand(double leftWheel, double rightWheel){
    int leftTick = leftWheel * ticks_per_meter;
    int rightTick = rightWheel * ticks_per_meter;

    minik_ros_wrapper::minikSetVelocityMsg msg;

    vector<int> motorID;
    motorID.push_back(0);
    motorID.push_back(1);
    msg.motorID = motorID;

    vector<int> velocity;
    velocity.push_back(leftTick);
    velocity.push_back(rightTick);
    msg.velocity = velocity;

    velPub.publish(msg);
}
