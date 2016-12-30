#include "Section.h"




Section::Section(int ID, int ServoID1, int ServoID2, int ServoID3, int FlexSensorPin1)
{
  m_iID = ID;
  m_iServoID1 = ServoID1;
  m_iServoID2 = ServoID2;
  m_iServoID3 = ServoID3;
  m_iFlexSensorRight = FlexSensorPin1;
  m_iFlexSensorLeft = FlexSensorPin1 + 1;
  m_iLeftFlex = 0;
  m_iRightFlex = 0;
  m_iMotorLoadRight = 0;
  m_iMotorLoadLeft = 0;
  PIDSetpoint = 0.0;
  m_dCurrentPosition = 0.0;
  m_iNegativeAngle = 0;
  m_PIDOutput = 0;
  m_DesiredTension = 100;
  m_TensionPull = 0;


}

void Section::m_ServoSetup()
{
  //set both servos Clockwise and Counter-Clockwise limits to 0. this sets them in wheel mode and allows continuous rotation.
  for (int registerID = 6; registerID <= 9; registerID += 2)
  {
    ax12SetRegister2(m_iServoID1, registerID, 0); // write two bytes to memory (ADDR, ADDR+1)
    ax12SetRegister2(m_iServoID2, registerID, 0); // write two bytes to memory (ADDR, ADDR+1)
  }

  ax12SetRegister2(m_iServoID3, 6, 0);
  ax12SetRegister2(m_iServoID3, 8, 1023);
}


int Section::m_PIDLoop(double Setpoint)
{
  int DeadbandFlag = 0;
  int KP = 20;

 // Serial.println(m_iID);
 // Serial.println(Setpoint);
  
  //Calculate angle
  m_GetFlexPosition();

  //find error
  m_dError = Setpoint - m_dCurrentPosition;

  //create deadband, where both pulleys run in to create tension
  if (abs(m_dError) <= 3)
  {
    m_TensionPull = m_PIDOutput + m_DesiredTension;
    if(m_TensionPull > 1023)
    {
      m_TensionPull = 1023;
    }
    m_iNegativeAngle = 2;  //negative angle indicates deadband
    DeadbandFlag = 1;
  }
  //operating PID
  else
  {
    //Vary KP for 3 vs 4 length sections.  may also need changing depending on spine properties.
    if (m_iID == 2)
    {
      KP = 10;
    }
    else
    {
      KP = 10;
    }

    //calculate PID terms
    m_dIntSum = m_dIntSum + (m_dError * 0.01);
    m_dProportional = (KP * m_dError);
    m_dIntegral = (KI * m_dIntSum);
    m_dDerivative = (KD * ((m_dError - m_dLastError) / 0.01));

    //prevent integral windup
    if (m_dIntegral >= 250)
    {
      m_dIntegral = 250;
    }
    else if (m_dIntegral <= -250)
    {
      m_dIntegral = -250;
    }

    //find PID output
    m_PIDOutput = m_dProportional + m_dIntegral + m_dDerivative;

    //prevent PIDOutput from overflowing and reversing motor direction
    if (m_PIDOutput >= 1023)
    {
      m_PIDOutput = 1023;
    }

    //check if direction to turn is counterclockwise
    if (m_PIDOutput < 0)
    {
      //make PID output +ve again, motors cant take -ve control values
      m_PIDOutput = -m_PIDOutput;
      m_iNegativeAngle = 1;
    }
    else
    {
      m_iNegativeAngle = 0;
    }
  }

  //if clockwise movement, add 1024 to reverse motor output, then add PID output to the extensor.
  if (m_iNegativeAngle == 0)
  {
    ax12SetRegister2(m_iServoID1, AX_GOAL_SPEED_L, 1024 + (m_PIDOutput / 3));  //release at a third the torque to prevent unwinding
    ax12SetRegister2(m_iServoID2, AX_GOAL_SPEED_L, 1024 + m_PIDOutput);

  }
  else if (m_iNegativeAngle == 1) //if counter-clockwise motors run opposite direction
  {
    ax12SetRegister2(m_iServoID1, AX_GOAL_SPEED_L, m_PIDOutput);
    ax12SetRegister2(m_iServoID2, AX_GOAL_SPEED_L, (m_PIDOutput / 3));
  }
  else if (m_iNegativeAngle == 2) //if in deadband, do nothing/ pull both servos in to tense
  {
    ax12SetRegister2(m_iServoID1, AX_GOAL_SPEED_L, 0);
    ax12SetRegister2(m_iServoID2, AX_GOAL_SPEED_L, 0);
  }
  m_dLastError = m_dError;
  return DeadbandFlag;
}

void Section::m_brakeControl(int Position)  //brake control for specific sections
{
  SetPosition(m_iServoID3, Position);
}

void Section::m_GetFlexPosition()    //find current section angle
{
  m_iRightFlex = analogRead(m_iFlexSensorRight);  //current flex sensor readings
  m_iLeftFlex  = analogRead(m_iFlexSensorLeft);

//  Serial.print("Right Sensor:");    //debugging messages
//  Serial.println(m_iRightFlex);
//  Serial.print("left Sensor:");
//  Serial.println(m_iLeftFlex);

  if (m_iID == 1)  //different sections have different sensor characteristics. these were plotted with manual movement and excel
  {
    m_dCurrentPosition = 7 + ((m_iRightFlex * 1.2256) - 535) * 0.5 + ((m_iLeftFlex * -1.2789) + 757) * 0.5;
  }
  else if ( m_iID == 2)
  {
    if (m_iRightFlex > 545)
    {
      m_dCurrentPosition = (((m_iRightFlex * -2.0825) + 1212.5) * 0.5 + ((m_iLeftFlex * 1.7153) - 826.34) * 0.5) + 5;
    }
    else if (m_iLeftFlex > 520)
    {
      m_dCurrentPosition = (((m_iRightFlex * -2.0825) + 1212.5) * 0.5 + ((m_iLeftFlex * 1.7153) - 826.34) * 0.5) + 5;
    }
    else
    {
      m_dCurrentPosition = (((m_iRightFlex * -2.0825) + 1212.5) * 0.5 + ((m_iLeftFlex * 1.7153) - 826.34) * 0.5) + 5;
    }
  }
  else if (m_iID == 3)
  {
    if (m_iRightFlex > 710)
    {
      m_dCurrentPosition = (((m_iRightFlex * 2.2805) - 1480.3) * 0.8 + ((m_iLeftFlex * -1.7892) + 1264.8) * 0.2);
    }
    else if (m_iLeftFlex > 680)
    {
      m_dCurrentPosition = (((m_iRightFlex * 2.2805) - 1480.3) * 0.2 + ((m_iLeftFlex * -1.7892) + 1264.8) * 0.8);
    }
    else
    {
      m_dCurrentPosition = (((m_iRightFlex * 2.2805) - 1480.3) * 0.5 + ((m_iLeftFlex * -1.7892) + 1264.8) * 0.5);
    }
  }
 // Serial.println(m_dCurrentPosition);
 // Serial.println(" ");
}

void Section::m_GetTension()//reads current motor tension. doesnt work very well on AX-12As
{
  m_iMotorLoadRight = ax12GetRegister(m_iServoID1, AX_PRESENT_LOAD_L, 2);
  m_iMotorLoadLeft =  ax12GetRegister(m_iServoID2, AX_PRESENT_LOAD_L, 2);
  Serial.print("m_iMotorLoadRight:");
  Serial.println(m_iMotorLoadRight);
  Serial.print("m_iMotorLoadLeft:");
  Serial.println(m_iMotorLoadLeft);
}

void Section::m_ReleaseServoTension(int AntagonistServo, int DesiredTension)  //experimental function to maintain stiffness in movement. currently non-functional
{
  //find the current motor tensions.
  m_GetTension();
  if (AntagonistServo == RIGHT_SERVO_ANTAG)
  {

    if (m_iMotorLoadRight >= 1024 + DesiredTension) //tension is similar to motor speed, 0-1023 one way, 1024-2047 the other way, just to confuse. why not two seperate registers?
    {
      ax12SetRegister2(m_iServoID2, AX_GOAL_SPEED_L, 1024 + 100); //if the motor load on the antagonistic servo is greater than the tension setting, spin it out to release that tension
      Serial.println("Right release");
    }
    else
    {
      ax12SetRegister2(m_iServoID2, AX_GOAL_SPEED_L, 0); //if the motor load is lower than the tension setting, hold position.
    }
  }
  else
  {
    if (m_iMotorLoadLeft >= 1024 + DesiredTension)
    {
      ax12SetRegister2(m_iServoID1, AX_GOAL_SPEED_L, 100); //if the motor load on the antagonistic servo is greater than the tension setting, spin it out to release that tension
      Serial.println("Left release");
    }
    else
    {
      ax12SetRegister2(m_iServoID1, AX_GOAL_SPEED_L, 0); //if the motor load is lower than the tension setting, hold position.
    }
  }

}















