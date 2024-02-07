#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <multi_dynamic/multi_util.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "publish_goals");

  ros::NodeHandle nh;

  /*
  ros::Publisher pathPublisher = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base_simple/goal", 1, true);

  move_base_msgs::MoveBaseActionGoal actionGoal;
  actionGoal.goal.target_pose.header.frame_id = "map";

  actionGoal.target_pose.pose.position.x = 20.0;
  actionGoal.target_pose.pose.position.y = 3.0;
  actionGoal.target_pose.pose.orientation.w = 1.0;
  */

  
  ros::Publisher goalPub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, true);
  geometry_msgs::PoseStamped goal;

  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();;

  goal.pose.position.x = 20.0;
  goal.pose.position.y = 3.0;
  goal.pose.orientation.w = 1.0;

  
  ros::Rate loopRate(10);
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 10);
  geometry_msgs::Twist vel_msg;
  while(ros::ok()){
    vel_msg.linear.x = 1.0;
    vel_msg.angular.z = 0.0;

    //cout << "Publico: " << vel_msg.linear.x << ", " << vel_msg.angular.z << endl;

    cmd_vel_pub.publish(vel_msg);
  //goalPub.publish(goal);

    loopRate.sleep();
  }



  return 0;
}