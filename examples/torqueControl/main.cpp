//
//  main.cpp
//  lib-robot
//
//  Created by Melanie Jouaiti on 07/07/2018.
//  Copyright © 2018 Melanie Jouaiti. All rights reserved.
//

#include "../../src/Robot.h"
#include <unistd.h>

int main(int argv, char** argc)
{
    Robot robot("/opt/MICO2SDK/API/Kinova.API.USBCommandLayerUbuntu.so");
    
    robot.startTorqueControl();
    for(unsigned int i = 0; i < 200; i++)
    {
        robot.setTorque(2, 0.7);
        usleep(1000);
    }
}
