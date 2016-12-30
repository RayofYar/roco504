#ifndef MOTION_H
#define MOTION_H

#define BRAKEUP 600      //positions for ventral actuator
#define BRAKEDOWN 750
#define BRAKEBACK 1023

#include <Arduino.h> //needed for Serial.println
#include "Section.h"

//variables for Serpentine movement. as the algorithm generates one set of points each time
//the variables must not be reset

int angleQuadrant = 1; //current phase of the sine waves
double sineAngle1;     //current Magnitude for each section
double sineAngle2;
double sineAngle3;
double Speed = 0.01;   //speed to move through the sine wave, size of increments

int Section1Angle;     //current target angle for each section
int Section2Angle;
int Section3Angle;

void forwardsSerpentine(Section* pSection1, Section* pSection2, Section* pSection3)
{ //sine wave 0-180. out of phase for each section to produce serpentine motion.
  
  Section1.m_brakeControl(BRAKEUP);  //deactivate any ventral actuators
  Section2.m_brakeControl(BRAKEUP);
  Section3.m_brakeControl(BRAKEUP);

  //wave generation
  switch (angleQuadrant)  //shifts through sine wave quadrants
  {
  case 1:  //step through quadrant one 
    Section1Angle = (asin(sineAngle1) * (180 / PI)) + 90;  
    sineAngle1 += Speed;
    Section2Angle = (asin(sineAngle2) * (180 / PI)) + 90;
    sineAngle2 -= Speed;
    Section3Angle = (asin(sineAngle3) * (180 / PI)) + 90;
    sineAngle3 -= Speed;
    if (sineAngle1 >= 0.2)  //move to quadrant two
    {
      angleQuadrant = 2;
    }
    break;

  case 2:  //step through quadrant one
    Section1Angle = (asin(sineAngle1) * (180 / PI)) + 90;
    sineAngle1 -= Speed;
    Section2Angle = (asin(sineAngle2) * (180 / PI)) + 90;
    sineAngle2 -= Speed;
    Section3Angle = (asin(sineAngle3) * (180 / PI)) + 90;
    sineAngle3 += Speed;
    if (sineAngle1 < 0.01)  //move to quadrant three
    {
      angleQuadrant = 3; 
    }
    break;

  case 3:    //step through quadrant three
    Section1Angle = (asin(sineAngle1) * (180 / PI)) + 90;
    sineAngle1 -= Speed;
    Section2Angle = (asin(sineAngle2) * (180 / PI)) + 90;
    sineAngle2 += Speed;
    Section3Angle = (asin(sineAngle3) * (180 / PI)) + 90;
    sineAngle3 += Speed;
    if (sineAngle1 < -0.2)  //move to quadrant four
    {
      angleQuadrant = 4;  
    }
    break;

  case 4:  //step through quadrant four
    Section1Angle = (asin(sineAngle1) * (180 / PI)) + 90;
    sineAngle1 += Speed;
    Section2Angle = (asin(sineAngle2) * (180 / PI)) + 90;
    sineAngle2 += Speed;
    Section3Angle = (asin(sineAngle3) * (180 / PI)) + 90;
    sineAngle3 -= Speed;
    if (sineAngle1 >= -0.01)  //return to quadrant one
    {
      angleQuadrant = 1;
    }
    break;

  default:  //default to straight snake
    Section1Angle = 90;
    Section2Angle = 90;
    Section3Angle = 90;
  }

  //  Serial.print("Section1Angle:");  //debug messages
  //  Serial.print(Section1Angle);
  //  Serial.print("       ");
  //  Serial.print("Section2Angle:");
  //  Serial.print(Section2Angle);
  //  Serial.print("       ");
  //  Serial.print("Section3Angle:");
  //  Serial.println(Section3Angle);
  //  delay(500);
  
  //run through each section PID loop. run each PID till in position
  int Section1Flag = 0;    //is section in position?
  int Section2Flag = 0;
  int Section3Flag = 0;
  do
  {
    Section1Flag = pSection1->m_PIDLoop(Section1Angle);
  } 
  while (Section1Flag != 1);
  do
  {
    Section2Flag = pSection2->m_PIDLoop(Section2Angle);
  } 
  while (Section2Flag != 1);
  do
  {
    Section3Flag = pSection3->m_PIDLoop(Section3Angle);
  } 
  while (Section3Flag != 1);

}





void forwardsConcertina() //curls and plants the rear brake, pushing forward from it.
{
  Section1.m_brakeControl(BRAKEDOWN);         //front brake down
  delay(1000);

  Section2Flag = 0;
  Section3Flag = 0;
  do
  {
    Section2Flag = Section2.m_PIDLoop(70);    //curl up
  } 
  while (Section2Flag != 1);
  do
  {
    Section3Flag = Section3.m_PIDLoop(110);  
  } 
  while (Section3Flag != 1);

  Section3.m_brakeControl(BRAKEDOWN);        //rear brake down, front brake up
  Section1.m_brakeControl(BRAKEUP);
  delay(1000);

  Section2Flag = 0;
  Section3Flag = 0;
  do
  {
    Section2Flag = Section2.m_PIDLoop(90);   //uncurl, pushing off from the rear
  } 
  while (Section2Flag != 1);
  do
  {
    Section3Flag = Section3.m_PIDLoop(90);
  } 
  while (Section3Flag != 1);

  Section3.m_brakeControl(BRAKEUP);          //rear brale up again
  delay(1000);  
} 






void forwardsSidewinder()
{
  Section1.m_brakeControl(BRAKEUP); //brakes ready
  Section2.m_brakeControl(BRAKEUP);
  Section3.m_brakeControl(BRAKEUP);

  Section1.m_brakeControl(BRAKEDOWN);
  Section3.m_brakeControl(BRAKEDOWN);   //front&rear brakes down
  delay(100);                           //give brakes time to move

  Section2Flag = 0;                          //push center section to one side
  do
  {
    Section2Flag = Section2.m_PIDLoop(70);
  } 
  while (Section2Flag != 1);
  Section3Flag = 0;
  do
  {
    Section3Flag = Section3.m_PIDLoop(70);
  } 
  while (Section3Flag != 1);

  Section2.m_brakeControl(BRAKEDOWN);
  Section1.m_brakeControl(BRAKEUP);       //middle section brake down, front brake up
  delay(100);

  Section2Flag = 0;
  do
  {
    Section2Flag = Section2.m_PIDLoop(90);        //move front section to line with middle
  } 
  while (Section2Flag != 1);


  Section1.m_brakeControl(BRAKEDOWN);
  Section3.m_brakeControl(BRAKEUP);       //front brake down, rear break up
  delay(100);

  Section3Flag = 0;
  do
  {
    Section3Flag = Section3.m_PIDLoop(90);        //bring rear section up to front
  } 
  while (Section3Flag != 1);

  Section1.m_brakeControl(BRAKEUP);       //all brakes up, in new position, ready to start again
  Section2.m_brakeControl(BRAKEUP);
  delay(100);

}

void forwardsRectilinear()  //This section requires that an flexible, inextensible connection exists between ventral actuators
{
  //curl front section, relax rear
  Section1.m_brakeControl(BRAKEUP);
  Section2.m_brakeControl(BRAKEBACK);
  Section3.m_brakeControl(BRAKEUP);
  delay(1000);
  //curl rear section, relax front
  Section1.m_brakeControl(BRAKEBACK);
  Section2.m_brakeControl(BRAKEUP);
  Section3.m_brakeControl(BRAKEBACK);
  delay(1000);
}

void backwards()
{
  //reverse sine wave?
}
void left()
{
  //adds offset to sine wave to create left turn?
}

void right()
{
  //adds offset to sinewave to create right turn?
}



#endif

