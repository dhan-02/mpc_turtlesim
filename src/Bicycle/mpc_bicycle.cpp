# include <ros/ros.h>
# include <math.h>
# include <turtlesim/Pose.h>
# include <geometry_msgs/Twist.h>
# include "ipopt_solver.hpp"
extern int N;
extern float dt;
extern int L;
extern float goal_x,goal_y;
extern float v_limit,sa_limit;
extern float msg_linear,msg_angular;
extern float obs_x,obs_y;
extern float x_now,y_now,theta_now,vel_now,steer_angle_now;
using namespace std;
float distance(float x1, float x2, float y1, float y2)
{
     //std::cout<<x1<<" "<<x2<<" "<<y1<<" "<<y2<<" ";
     return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
void pose_runner_Callback(const turtlesim::Pose::ConstPtr& msg)
{
     x_now = msg->x;
     y_now = msg->y;
     theta_now = msg->theta;
     vel_now = msg->linear_velocity;
     steer_angle_now = asin(msg->angular_velocity*L/vel_now);
}
void pose_obs_Callback(const turtlesim::Pose::ConstPtr& msg)
{
     obs_x = msg->x;
     obs_y = msg->y;
}
void input()
{
     cout<<"**************** WElCOME ********************\n\n";
     cout<<"Enter Goal x: "; std::cin>>goal_x; std::cout<<"\n\n";
     cout<<"Enter Goal y: "; std::cin>>goal_y; std::cout<<"\n\n";
     cout<<"Processing.........\n\n";
}
int main(int argc, char **argv)
{
     ros::init(argc, argv, "turtlepath_node");
     char key = 'Y';
     ros::NodeHandle n;
     n.getParam("/turtle_mpc/N",N);
     n.getParam("/turtle_mpc/dt",dt);
     n.getParam("/turtle_mpc/L",L);
     n.getParam("/turtle_mpc/v_limit",v_limit);
     n.getParam("/turtle_mpc/sa_limit",sa_limit);
     ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1000);
     ros::Rate loop_rate(100);
     geometry_msgs::Twist vel;
     ros::Subscriber sub = n.subscribe("turtle2/pose", 1000, pose_runner_Callback);
     ros::Subscriber sub2 = n.subscribe("turtle1/pose", 1000, pose_obs_Callback);
     while(key=='Y'||key=='y')
     {
          input();
          while (ros::ok())
          {
               get_started();
               vel.linear.x = msg_linear;
               vel.angular.z = msg_angular;
               vel.linear.y = vel.linear.z = vel.angular.x = vel.angular.y = 0;
               if(distance(x_now,goal_x,y_now,goal_y)<0.5)
               {
                    break;
               }
               chatter_pub.publish(vel);
               ros::spinOnce();
               loop_rate.sleep();
          }
          vel.linear.x = vel.angular.z = 0;
          chatter_pub.publish(vel);
          std::cout<<"Turtle time over\n\n****************************************\n\n";
          std::cout<<"Do you want to go again (Y/N) : "; std::cin>>key; std::cout<<"\n\n";
     }
     std::cout<<N;
     std::cout<<L;
     std::cout<<sa_limit;
     return 0;
}

