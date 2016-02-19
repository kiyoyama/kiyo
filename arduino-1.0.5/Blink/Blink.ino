#include <ros.h>
#include <geometry_msgs/Pose.h>

ros::NodeHandle nh;

//geometry_msgs::Pose pose_msgs;
//ros::Subscriber pose("pose",&pose_msgs);


void messageCb(const geometry_msgs::Pose& pose_msg)
{
  digitalWrite(13, HIGH-digitalRead(13));
  float x=pose_msg.position.x;
  //pose_msgs.position.x;
 }
ros::Subscriber<geometry_msgs::Pose> sub("toggle_led", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(1000);
}
