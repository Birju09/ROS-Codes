// ConsoleApplication4.cpp : Defines the entry point for the console application.
// This code is a very simple implementation of the control of differential drive robot to reach from point A to point B
// the point is fetched from the clicked point publisher in Rviz
// For obstacle avoidance it performs a maneuver of turning 45 and travelling 0.5 meters in the rotated direction and do the path planninf again
// Author : Birju Vachhani 
// Time stamp: 1st June, 2018, 8:57pm
// Rutgers University , Zou Lab, Department of Mechanical and Aerospace Engineering, Piscatway, NJ08854

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/Range.h>
#include <vector>

//class for positions
class Position
{
public:
	float x;
	float y;
	float th;
	
	Position()
	{

	}
	void setPosition(float x, float y, float th)
	{
		this->x = x;
		this->y = y;
		this->th = th;
	}
};

struct min
{
	float minimum;
	int index;
};

//global objects
Position cur;
Position initial;
Position pos;
std::vector<float> sonarlist;


//build the sonars
void buildsonar(int n)
{
	for (int i=0;i<n;i++)
	{
		sonarlist.push_back(10.0);
	}
	ROS_INFO("Sonar array built");
}

//callback to set the current position
void callback(const geometry_msgs::PoseStamped &data)
{
	float yaw;
	yaw = (2 * ((data.pose.orientation.x * data.pose.orientation.w) + (data.pose.orientation.y * data.pose.orientation.z))) / (1 - 2 * (pow(data.pose.orientation.y, 2) * pow(data.pose.orientation.z, 2)));
	cur.setPosition(data.pose.position.x, data.pose.position.y, yaw);
}

//callback to get the initial position
void callback_init(const geometry_msgs::PoseStamped& data)
{
	float yaw;
	yaw = (2 * ((data.pose.orientation.x * data.pose.orientation.w) + (data.pose.orientation.y * data.pose.orientation.z))) / (1 - 2 * (pow(data.pose.orientation.y, 2) * pow(data.pose.orientation.z, 2)));
	initial.setPosition(data.pose.position.x, data.pose.position.y, yaw);
}

//function to calculate the distance between two points
float distance(Position &point1, Position &point2)
{
	return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}


//Sonar callbacks
void callback1(const sensor_msgs::Range& son1data)
{
	sonarlist[0] = son1data.range;
	
}
void callback2(const sensor_msgs::Range& son2data)
{
	sonarlist[1] = son2data.range;
	
}
void callback3(const sensor_msgs::Range& son3data)
{
	sonarlist[2] = son3data.range;
	
}
void callback4(const sensor_msgs::Range& son4data)
{
	sonarlist[3] = son4data.range;
}


//maneuver function
void turn(int sign)
{
	ros::Time begin = ros::Time::now();
	geometry_msgs::Twist twist;
	ros::Publisher pub = n.advertise("cmd_vel", geometry_msgs::Twist, 10);
	
	
	while ((ros::Time::now() - begin).toSec() < 2)
	{
		twist.angular.z = 0.5 * sign;
		twist.linear.x = 0.0;
		pub.publish(twist);
	}
	ros::Time begin = ros::Time::now();
	while ((ros::Time::now() - begin).toSec() < 2)
	{
		twist.angular.z = 0.0;
		twist.linear.x = 0.5;
		pub.publish(twist);
	}
}


//minimum function
min minlist(std::vector<float> &sonarlist)
{
	min minsonar;
	float a = sonarlist[0];
	
	for (int it=0;it<sonarlist.size();it++)
	{
		if (sonarlist[it] < a)
		{
			a = sonarlist[it];
			minsonar.index = it;
		}
			
	}
	minsonar.minimum = a;
	return minsonar;
}


//maneuver direction
void do_maneuver()
{
	min minsonar;
	minsonar = minlist(sonarlist);
	if (minsonar.minimum == 1 || minsonar.minimum == 2)
	{
		turn(1);
	}
	else if (minsonar.minimum == 4 || minsonar.minimum == 3)
	{
		turn(-1);
	}
	ROS_INFO("maneuver is completed");
	getpoint();
}



//function to run the controller until the goal is reached
void control_run(const geometry_msgs::PoseStamped& msg)
{
	ROS_INFO("Starting the controller now");
	
	pos.setPosition(msg.pose.position.x, msg.pose.position.y, 0.0);
	ros::Publisher pub = n.advertise("cmd_vel", geometry_msgs::Twist, 10);
	float angle, curr_angle;
	curr_angle = cur.th;
	angle = atan((cur.y - pos.y) / (cur.x - pos.x));
	while (distance(cur, pos) > 0.5)
	{
		
		if (obstacle_range() < 1.0)
		{
			ROS_WARN("Obstacle on the way, performing maneuver now");
			do_maneuver();	
			getpoint();
		}
		geometry_msgs::Twist twist;
		
		while ((curr_angle - angle) > 2.0)
		{
			curr_angle = cur.th;
			ROS_INFO("current angle is %f", curr_angle);
			twist.angular.z = 0.5;
			pus.publish(twist);
			ros::spinOnce();
		}
		twist.linear.x = 1.0;
		twist.angular.z = 0.0;
		pub.publish(twist);
		ros::spinOnce();
		ROS_INFO("Distance is %f", distance(cur,pos));
	}
	ROS_INFO("Desitination has arrived");
}




//function to calculate the distance from the obstacle
float obstacle_range()
{
	ros::Subscriber sonar1 = n.Subscriber("sonar_1", 10, callback1);
	ros::Subscriber sonar2 = n.Subscriber("sonar_2", 10, callback2);
	ros::Subscriber sonar3 = n.Subscriber("sonar_3", 10, callback3);
	ros::Subscriber sonar4 = n.Subscriber("sonar_4", 10, callback4);

	float obstacle_distance = fmin(fmin(son1,son2),fmin(son3,son4));
	return obstacle_distance;
}




//function to get the new initial point if a maneuver to avoid the obstacle has to be carried out 
void getpoint()
{
	ros::Subscriber sub;
	sub = n.subscribe("/slam_out_pose", 1, callback_init);
	sub.shutdown();	
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "Controller");
	ros NodeHandle n;
	buildsonar(4);
    ros::Subscriber sub1 = n.subscribe("/slam_out_pose", 10, callback);
	ros::Subscriber sub2 = n.subscribe("clicked_point", 10, control_run);

	while (ros::ok())
	{
		ros::spin();
	}
    return 0;
}


