#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>

std::string robot;



void poseCallback(const gazebo_msgs::ModelStates& msg){

  if (robot=="robot2"){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg.pose[2].position.x, msg.pose[2].position.y, 0.0) );
    tf::Quaternion q;
    q.setX(0);q.setY(0);q.setZ(msg.pose[2].orientation.z);q.setW(msg.pose[2].orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot));
  }
  else{
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(msg.pose[1].position.x, msg.pose[1].position.y, 0.0) );
      tf::Quaternion q;
      q.setX(0);q.setY(0);q.setZ(msg.pose[1].orientation.z);q.setW(msg.pose[1].orientation.w);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot));
  };
};

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_pose _broadcaster");
  if (argc != 2){ROS_ERROR("need the robot name (robot1 or robot2) as argument"); return -1;};
  robot = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("gazebo/model_states", 100, &poseCallback);

  ros::spin();
  return 0;
}