/*****************************************************
 * Title:         Soft Robotic Snake Control
 * Description:   Runs basic control for soft robotic snake
 * Authors:       Ryan Draper, Jane Sheard, Matt Troughton
 * Date:          30/12/2016
 * Version:       1.0
 *****************************************************/

//import ax12 library to send DYNAMIXEL commands
#include <ax12.h>

#include "Motion.h"  

#define AX_GOAL_SPEED_L 32  //lower register for torque percentage and direction 

//double Input = 90;     //a basic input for control functions. Used for debugging. Can be changed by reading from PC.

Section Section1 = Section(1, 5, 3, 1, 0);  //initialises three sections, contains the ax12-A servo IDs
Section Section2 = Section(2, 7, 6, 4, 2);
Section Section3 = Section(3, 9, 8, 10, 4);

void setup()
{
  Serial.begin(115200);                 //Baud rate for comunication with PI

  delay(1000);                          //short set of movements for ventral actuators. signal that board is active.
  Section1.m_brakeControl(BRAKEUP);
  delay(500);
  Section1.m_brakeControl(BRAKEDOWN);
  delay(500);
  Section1.m_brakeControl(BRAKEUP);
  delay(1000);

  Section1.m_ServoSetup();              //set up of all Servos
  Section2.m_ServoSetup();
  Section3.m_ServoSetup();

}

void loop()
{

  String msg = Serial.readString();  //read messages from the PI
  Serial.print(msg + "\n");

  if (msg.startsWith("F"))  //More button controls can be added for different commands.
  {
    //forwards routine. Choose one to operate. 
    
    //forwardsConcertina();    
    //forwardsRectilinear();
    //forwardsSidewinder();
    //forwardsSerpentine(&Section1, &Section2, &Section3);

  }
  if (msg.startsWith("B"))
  {
    // stop routine
  }
  else
  {
    //in case message mess up
  }

  //Debugging. Serial Reads new input values. can manually set positions for each section
  //Section3.m_PIDLoop(Input);
  //  if (Serial.available() > 0);
  //  {
  //    test = Serial.parseInt(); //read int or parseFloat for ..float...
  //    if (test != 0)
  //    {
  //      Input = test;
  //    }
  //  }

}










