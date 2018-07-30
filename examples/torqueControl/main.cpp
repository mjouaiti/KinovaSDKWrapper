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

#include "../../src/Robot.h"
#include <unistd.h>

int main(int argv, char** argc)
{
    Robot robot("/opt/kinova/API/Kinova.API.USBCommandLayerUbuntu.so");
    
    robot.startTorqueControl();
    for(unsigned int i = 0; i < 200; i++)
    {
        robot.setTorque(2, 0.7);
        usleep(1000);
    }
}
