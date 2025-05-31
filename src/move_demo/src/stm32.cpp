// File: src/stm32_odom_publisher.cpp

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class STM32OdomSimulator {
public:
  STM32OdomSimulator() {
    ros::NodeHandle pnh("~");
    pnh.param("model_name",     model_name_,     std::string("mycar"));
    pnh.param("odom_topic",     odom_topic_,     std::string("/odom_stm32"));
    pnh.param("odom_frame_id",  odom_frame_id_,  std::string("odom"));
    pnh.param("base_frame_id",  base_frame_id_,  std::string("base_link"));
    pnh.param("publish_tf",     publish_tf_,     true);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    sub_      = nh_.subscribe("/gazebo/model_states", 10,
                              &STM32OdomSimulator::modelStatesCb, this);

    // ROS_INFO("STM32 Odom Simulator started: model=[%s] → topic=[%s], publish TF=%s",
    //          model_name_.c_str(), odom_topic_.c_str(),
    //          publish_tf_ ? "true" : "false");
  }

private:
  void modelStatesCb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // Find the index of our model
    auto it = std::find(msg->name.begin(), msg->name.end(), model_name_);
    if (it == msg->name.end()) {
      ROS_WARN_THROTTLE(10, "Model '%s' not found. Make sure it is spawned in Gazebo with this name.",
                        model_name_.c_str());
      return;
    }
    int idx = std::distance(msg->name.begin(), it);

    // Extract pose and twist
    const auto& pose  = msg->pose[idx];
    const auto& twist = msg->twist[idx];

    // Build and publish the Odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp    = ros::Time::now();
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id  = base_frame_id_;
    odom.pose.pose       = pose;
    odom.twist.twist     = twist;
    odom_pub_.publish(odom);

    // Optionally broadcast TF from odom → base_link
    if (publish_tf_) {
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(pose.position.x,
                                      pose.position.y,
                                      pose.position.z));
      tf::Quaternion q(pose.orientation.x,
                       pose.orientation.y,
                       pose.orientation.z,
                       pose.orientation.w);
      transform.setRotation(q);
      tf_broadcaster_.sendTransform(
        tf::StampedTransform(transform,
                             odom.header.stamp,
                             odom_frame_id_,
                             base_frame_id_));
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher      odom_pub_;
  ros::Subscriber     sub_;
  tf::TransformBroadcaster tf_broadcaster_;

  std::string model_name_, odom_topic_;
  std::string odom_frame_id_, base_frame_id_;
  bool        publish_tf_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "stm32_odom_publisher");
  STM32OdomSimulator node;
  ros::spin();
  return 0;
}
