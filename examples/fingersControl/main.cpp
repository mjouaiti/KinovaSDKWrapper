//
//  main.cpp
//  lib-robot
//
//  Created by Melanie Jouaiti on 07/07/2018.
//  Copyright © 2018 Melanie Jouaiti. All rights reserved.
//

#include "../../src/Robot.h"

int main(int argv, char** argc)
{
    Robot robot("../../lib/Kinova.API.USBCommandLayerUbuntu.so", "config.txt");
    
    robot.setFingerVelocity(1, 0.4);
    robot.setFingerPosition(2, 45);
}
