#include <ax12.h>

#ifndef SECTION_H
#define SECTION_H

//#define KP  20  //PID gains
#define KI 1000
#define KD 0  
#define RIGHT_SERVO_ANTAG 0
#define LEFT_SERVO_ANTAG 1

#define AX_GOAL_SPEED_L 32

class Section  //Section variables
{
  public:

    Section(int ID, int ServoID1, int ServoID2, int ServoID3, int FlexSensorPin1);

    int    m_PIDLoop(double Setpoint);
    void   m_ReleaseServoTension(int AntagonisticServo, int DesiredTension);
    void   m_GetFlexPosition();
    void   m_GetSerialData();
    void   m_GetTension();
    void   m_ServoSetup();
    void   m_GetServoPositions();
    void   m_brakeControl(int Position);

    int    m_iID;
    int    m_iServoID1;
    int    m_iServoID2;
    int    m_iServoID3;
    int    m_iLeftFlex;
    int    m_iRightFlex;
    int    m_iFlexSensorRight;
    int    m_iFlexSensorLeft;
    
    int    m_iMotorLoadRight;
    int    m_iMotorLoadLeft;
    int    m_iNegativeAngle;
     
    
    int PIDSetpoint;
    int m_dCurrentPosition;
    int m_DesiredTension;
    int m_TensionPull;
    

    int m_dError;
    int m_dLastError;
    int m_PIDOutput;
    double m_dIntSum;
    double m_dProportional;
    double m_dIntegral;
    double m_dDerivative;

};


extern Section Section1;
extern Section Section2;
extern Section Section3;


#endif


