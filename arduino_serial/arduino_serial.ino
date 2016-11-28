#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle n;

void serialCallback(const std_msgs::UInt16& msg)
{  
  if (msg.data == 1)
  {
    // execute routine
    Serial.print("FORWARDS \n");
  }
  if (msg.data == 2)
  {
    // execute routine
    Serial.print("BACKWARDS \n");
  }
  if (msg.data == 3)
  {
    // execute routine
    Serial.print("LEFT \n");
  }
  if (msg.data == 4)
  {
    // execute routine
    Serial.print("RIGHT \n");
  }
}

ros::Subscriber<std_msgs::UInt16> c_sub ("commands_serial", &serialCallback);

void setup()
{
  Serial.begin(57600);
  n.initNode();
  n.subscribe(c_sub);
}

void loop()
{
  n.spinOnce();
  delay(1);
}
