//
//  main.cpp
//
//  Kinova SDK Wrapper
//  Copyright (C) 2018  Université de Lorraine - CNRS
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//  Created by Melanie Jouaiti on 07/07/2018.
//

#include <iostream>
#include "../../src/Robot.h"
#include <unistd.h>


int main(int argv, char** argc)
{
    Robot* robot = new Robot("/opt/MICO2SDK/API/Kinova.API.USBCommandLayerUbuntu.so");
    robot->initFingers();
    std::cout << robot->getGripperStatus();

//positive value opens the fingers, negative value closes them
    for(int i = 0; i < 10000; i++)
    {
	robot->setFingerVelocity(1, 3000);
        robot->setFingerVelocity(2, 3000);
        usleep(1000);
    }
}
