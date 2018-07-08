//
//  main.cpp
//  lib-robot
//
//  Created by Melanie Jouaiti on 07/07/2018.
//  Copyright © 2018 Melanie Jouaiti. All rights reserved.
//

#include <iostream>
#include "../../src/Robot.h"


int main(int argv, char** argc)
{
    Robot* robot = new Robot("/opt/MICO2SDK/API/Kinova.API.USBCommandLayerUbuntu.so");
    
    std::cout << "control fingers" << std::endl;
    //robot.setFingerVelocity(1, 0.4);
    //robot.setFingerPosition(2, 45);
    robot->setAngularControl();
    std::cout << "control fingers" << std::endl;
}
