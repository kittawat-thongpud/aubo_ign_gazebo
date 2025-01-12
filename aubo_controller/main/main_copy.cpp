#include <iostream>

#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include "util.h"

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

#define SERVER_HOST "192.168.1.101"
#define SERVER_PORT 8899

int main()
{
   
    ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    //Login
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"login successful."<<std::endl;
    }
    else
    {
        std::cerr<<"login failed."<<std::endl;
    }
    
    
    /** If the real robot arm is connected, the arm needs to be initialized.**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //Tool dynamics parameter
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**Tool dynamics parameter**/,
                                               6        /*Collision level*/,
                                               true     /*Whether to allow reading poses defaults to true*/,
                                               true,    /*Leave the default to true */
                                               1000,    /*Leave the default to 1000 */
                                               result); /*Robot arm initialization*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"Robot arm initialization succeeded."<<std::endl;
    }
    else
    {
        std::cerr<<"Robot arm initialization failed."<<std::endl;
    }

    //Initialize motion properties
    robotService.robotServiceInitGlobalMoveProfile();

    /** Interface call: Set the maximum acceleration of the articulated motion ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 10.0/180.0*M_PI;
    jointMaxAcc.jointPara[1] = 10.0/180.0*M_PI;
    jointMaxAcc.jointPara[2] = 10.0/180.0*M_PI;
    jointMaxAcc.jointPara[3] = 10.0/180.0*M_PI;
    jointMaxAcc.jointPara[4] = 10.0/180.0*M_PI;
    jointMaxAcc.jointPara[5] = 10.0/180.0*M_PI;   ////The interface requires the unit to be radians
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    /** Interface call: set the maximum speed of articulated motion ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 10.0/180.0*M_PI;
    jointMaxVelc.jointPara[1] = 10.0/180.0*M_PI;
    jointMaxVelc.jointPara[2] = 10.0/180.0*M_PI;
    jointMaxVelc.jointPara[3] = 10.0/180.0*M_PI;
    jointMaxVelc.jointPara[4] = 10.0/180.0*M_PI;
    jointMaxVelc.jointPara[5] = 10.0/180.0*M_PI;   ////The interface requires the unit to be radians
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);


    /** Interface call: Initialize motion properties ***/
    robotService.robotServiceInitGlobalMoveProfile();

    ret = robotService.rootServiceRobotControl(aubo_robot_namespace::RobotControlCommand::ClearSingularityOverSpeedAlarm);

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
        std::cerr<<"clear singularityOverSpeedAlarm success.　ret:"<<ret<<std::endl;
    else
        std::cerr<<"clear singularityOverSpeedAlarm failed.　ret:"<<ret<<std::endl;

    
    toolDynamicsParam.positionX = 0.0;
    toolDynamicsParam.positionY = 0.0;
    toolDynamicsParam.positionZ = 0.0;
    toolDynamicsParam.payload = 0.5;
    ret = robotService.robotServiceGetToolDynamicsParam(toolDynamicsParam);
    robotService.robotServiceLeaveTcp2CanbusMode();
    //Preparation point
    double jointAngle[aubo_robot_namespace::ARM_DOF];
    Util::initJointAngleArray(jointAngle, 0.0, 0.0, 90.0/180.0*M_PI, 0.0, 90.0/180.0*M_PI, 0.0);
    ret = robotService.robotServiceJointMove(jointAngle, true);   //Joint movement to preparation point
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"Joint move failed.　ret:"<<ret<<std::endl;
    }
    else
        std::cerr<<"Joint move finish.　ret:"<<ret<<std::endl;
    
    for (float32 i=0.0; i<300;i++)
    {
        Util::initJointAngleArray(jointAngle, (i/10)/180.0*M_PI, 0.0, 90.0/180.0*M_PI, 0.0, 90.0/180.0*M_PI, 0.0);
        ret = robotService.robotServiceFollowModeJointMove(jointAngle);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"Joint "<<i<<" move "<<jointAngle[0]<<" failed.　ret:"<<ret<<std::endl;
        }
        else
            std::cerr<<"Joint "<<i<<" move "<<jointAngle[0]<<" finish.　ret:"<<ret<<std::endl;

    }
    for (float32 i=0.0; i<10000;i++)
    {
    }
    /*
    ret = robotService.robotServiceEnterTcp2CanbusMode();
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"Enter TCP2CanbusMode Fail.　ret:"<<ret<<std::endl;
    }
    else
        std::cerr<<"Enter TCP2CanbusMode finish.　ret:"<<ret<<std::endl;

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
        std::cerr<<"set payload success.　ret:"<<ret<<std::endl;
    else
        std::cerr<<"set payload failed.　ret:"<<ret<<std::endl;
    
    */
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);


    //Set the maximum acceleration of the end motion. Linear motion is the end motion.
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(0.5);    //Units m/s2
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(0.5);   //Units m/s

    robotService.robotServiceRobotShutdown();

    /** Interface call: logout　**/
    robotService.robotServiceLogout();

    return 0;
}

