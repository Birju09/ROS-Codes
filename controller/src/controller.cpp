// This code is a very simple implementation of the control of differential drive robot to reach from point A to point B
// the point is fetched from the clicked point publisher in Rviz
// For obstacle avoidance it performs a maneuver of turning 45 and travelling 0.5 meters in the rotated direction and do the path planninf again
// Author : Birju Vachhani 
// Time stamp: 1st June, 2018, 8:57pm
// Rutgers University , Zou Lab, Department of Mechanical and Aerospace Engineering, Piscatway, NJ08854

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/Range.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>

//class for positions
class Position
{
public:
	float x;
	float y;
	float th;
	friend float distance(Position &point1, Position &point2);
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

class Pubsub
{

public:
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Subscriber sub1 = n.subscribe("/slam_out_pose", 1, &Pubsub::callback, this);
    ros::Subscriber sonar1 = n.subscribe("/sonar1", 2, &Pubsub::callback1, this);
	ros::Subscriber sonar2 = n.subscribe("/sonar2", 2, &Pubsub::callback2, this);
	ros::Subscriber sonar3 = n.subscribe("/sonar3", 2, &Pubsub::callback3, this);
	ros::Subscriber sonar4 = n.subscribe("/sonar4", 2, &Pubsub::callback4, this);
	ros::Subscriber sub2 = n.subscribe("/clicked_point", 2, &Pubsub::control_run, this);
	

	//callback to set the current position

	void callback(const geometry_msgs::PoseStamped &data)
	{
        double roll,pitch,yaw;
		tf::Quaternion quater;
		tf::quaternionMsgToTF(data.pose.orientation, quater);
		tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);
		cur.setPosition(data.pose.position.x, data.pose.position.y, yaw);
	}

	//callback to get the initial position
	void callback_init(const geometry_msgs::PoseStamped& data)
	{
        double roll,pitch,yaw;
		tf::Quaternion quater;
		tf::quaternionMsgToTF(data.pose.orientation, quater);
		tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);
		initial.setPosition(data.pose.position.x, data.pose.position.y, yaw);
	}


	//Sonar callbacks
	void callback1(const sensor_msgs::Range& son1data)
	{
		sonarlist[0] = son1data.range;
        //ROS_INFO("distance is %f",sonarlist[0]);

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

	//main control run
	void control_run(const geometry_msgs::PointStamped& msg)
	{
		ROS_INFO("Starting the controller now");
        ros::Rate r(50);
		pos.setPosition(msg.point.x, msg.point.y, 0.0);
		float angle, curr_angle;
		curr_angle = cur.th;
        float sign=1.0;
		if (pos.x > cur.x)
		{
			sign = 1.0;
		}
		else
		{
			sign = -1.0;
		}
        int angsign = 1;
		int obsign= 1;
		while (distance(cur, pos) > 0.35)
		{
            //ROS_INFO("distance from obsacle is %f",obstacle_range());
            angle = atan((cur.y - pos.y) / (cur.x - pos.x));
			if (obstacle_range() < 40.0)
			{
				ROS_WARN("Obstacle on the way, performing maneuver now");
                if(cur.x  > pos.x)
                {
                    obsign = 1;
                }
                else
                {
                    obsign = -1;
                }
				do_maneuver(obsign);
				ros::spinOnce();
			}
			geometry_msgs::Twist twist;
            float kp=0.1,ki=0.01,error,error_sum=0.0;
			//std::cout << curr_angle - angle << std::endl;
			while (fabs(curr_angle - angle) > 0.035)
			{
                ros::spinOnce();
                angle = atan((cur.y - pos.y) / (cur.x - pos.x));
                if((curr_angle-angle) > 0.0)
                {
                    angsign = -1.0;
                }
                else if ((curr_angle-angle) < 0.0)
                {
                    angsign = 1.0;
                }
				curr_angle = cur.th;
                error = fabs(curr_angle - angle);
                error_sum = error_sum + error;
				//ROS_INFO("current angle absolute difference is %f", error);
				twist.angular.z = fmin(0.25,(error_sum*ki) + (kp*error))*angsign;
				pub.publish(twist);
                r.sleep();
				
			}
			twist.linear.x = -1.75 * sign;
			twist.angular.z = 0.0;
			pub.publish(twist);
			ros::spinOnce();
			//ROS_INFO("Distance is %f", distance(cur, pos));
		}
		ROS_INFO("Desitination has arrived");
		geometry_msgs::Twist twist;
		twist.linear.x = 0.0;
		twist.angular.z = 0.0;
		pub.publish(twist);

	}

	//maneuver direction
	void do_maneuver(int obsign)
	{
		min minsonar;
        ROS_INFO("In now");
		minsonar = minlist(sonarlist);
		ros::Time begin = ros::Time::now();
		geometry_msgs::Twist twist;
		while (obstacle_range() > 50)
		{
            ROS_INFO("In the loop of turn now and obsign is %d",obsign);
			twist.angular.z = 0.3;
			twist.linear.x = 0.0;
			pub.publish(twist);
		}
		begin = ros::Time::now();
		while ((ros::Time::now() - begin).toSec() < 11.5)
		{
			twist.angular.z = 0.0;
			twist.linear.x = 0.5 * obsign;
			pub.publish(twist);
		}	
		ROS_INFO("maneuver is completed");
		ros::spinOnce();
		
	}
    float average(int n)
    {
        float average = 0.0;
        for(int i=0;i<=4;i++)
        {
            average += sonarlist[n];
            ros::spinOnce();
        }
        return average;
    }
	//function to calculate the distance from the obstacle
	float obstacle_range()
	{
        float obstacle_distance = fmin(fmin(sonarlist[0], sonarlist[1]), fmin(sonarlist[2], sonarlist[3]));
		return obstacle_distance;
	}

	//function to calculate the distance between two points
	float distance(Position &point1, Position &point2)
	{
		return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
	}

	//minimum function
	min minlist(std::vector<float> &sonarlist)
	{
		min minsonar;
		float a = sonarlist[0];
		for (int it = 0; it<sonarlist.size(); it++)
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


};


//build the sonars
void buildsonar(int n)
{
	for (int i = 0; i<n; i++)
	{
		sonarlist.push_back(100.0);
	}
	ROS_INFO("Sonar array built");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Controller");

	buildsonar(4);
    
	Pubsub control;


	while (ros::ok())
	{
		ros::spin();
        
	}
	return 0;
}

