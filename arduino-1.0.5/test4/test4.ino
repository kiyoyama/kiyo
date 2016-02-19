#include <ros.h>
#include <geometry_msgs/Pose.h>

ros::NodeHandle nh;

geometry_msgs::Pose pose_msgs;
ros::Publisher pose("arm",&pose_msgs);

float x;
float y;
float z;

void messageCb(const geometry_msgs::Pose& pose_msgs)
{
  digitalWrite(13, HIGH-digitalRead(13));
   nh.loginfo("Program info");
   x = pose_msgs.position.x ;
   y = pose_msgs.position.y;
   z = pose_msgs.position.z;
   pose.publish(&pose_msgs);
 }
ros::Subscriber<geometry_msgs::Pose> sub("toggle_led", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pose);
}

void loop() {
  
  nh.spinOnce();
  delay(1);
}
