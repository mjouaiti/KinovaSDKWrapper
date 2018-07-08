//
//  main.cpp
//  lib-robot
//
//  Created by Melanie Jouaiti on 07/07/2018.
//  Copyright © 2018 Melanie Jouaiti. All rights reserved.
//

#include <stdio.h>

#include "../../src/Robot.h"

int main(int argv, char** argc)
{
    Robot robot("../../lib/Kinova.API.USBCommandLayerUbuntu.so", "config.txt");
    
    robot.setAngularControl();
    robot.setPosition({180, 200, 300, 0, 0, 0});
}
