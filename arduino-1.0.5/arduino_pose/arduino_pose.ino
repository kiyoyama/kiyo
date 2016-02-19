#include <ros.h>
#include <geometry_msgs/Pose.h>

ros::NodeHandle nh;

geometry_msgs::Pose pose_msgs;
ros::Publisher pose("pose",&pose_msgs);

void setup(){
  // put your setup code here, to run once:
  //Serial.begin(9600);
  nh.initNode();
  nh.advertise(pose);
}

void loop() {
  // put your main code here, to run repeatedly:
  pose_msgs.position.x = 1.0;
  pose_msgs.position.y = 2.0;
  pose_msgs.position.z = 3.0;
  pose.publish(&pose_msgs);
  nh.spinOnce();
  delay(1000);
}
