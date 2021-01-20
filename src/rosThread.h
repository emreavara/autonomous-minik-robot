#include <ros/ros.h>
#include <tf/tf.h>
#include <QDebug>
#include <QVector>
#include <QObject>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32MultiArray.h>
#include <minik_ros_wrapper/minikSetVelocityMsg.h>
#include "math.h"

using namespace std;

struct Obstacle{
public:
    double x;
    double y;
    double r;
};

class RosThread:public QObject
{
    Q_OBJECT

public:
    RosThread();
    ~RosThread();

private:
    ros::NodeHandle n;
    ros::Publisher velPub;
    ros::Subscriber odomSub;
    ros::Subscriber targetSub;
    ros::Subscriber obstacleSub;

    // Define the global variables and prototype functions here

    static const double wheelRad = 0.045; // in meters
    static const double robotRadius = 0.1; // in meters

    double travelDistance; //total travelled distance

    double robotX;  // in meters
    double robotY;  // in meters
    double robotTh; // in radians

    double targetX; // in meters
    double targetY; // in meters
    double targetTh; // in radians

    std::vector<Obstacle> obstacles;

    void demoLoop();
    void odomHandler(const nav_msgs::OdometryConstPtr &odomMsg);
    void targetHandler(const geometry_msgs::PoseConstPtr &targetMsg);
    void obstacleHandler(const std_msgs::Float32MultiArrayConstPtr &obstacleMsg);

    void sendVelocityCommand(double leftWheel, double rightWheel); // meters per second
    static const int ticks_per_meter = 10610;

    double _lastX;
    double _lastY;

public slots:
     void work();

};

