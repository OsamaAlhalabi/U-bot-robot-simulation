# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Demonstration of inverse kinematics using the "ikpy" Python module."""

import sys
import tempfile
try:
    import ikpy
    from ikpy.chain import Chain
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

import math
from controller import Supervisor, Robot

if ikpy.__version__[0] < '3':
    sys.exit('The "ikpy" Python module version is too old. '
             'Please upgrade "ikpy" Python module to version "3.0" or newer with this command: "pip install --upgrade ikpy"')

# Initialize the Webots Supervisor.
supervisor = Supervisor()
timeStep = int(supervisor.getBasicTimeStep())

# Create the arm chain from the URDF
filename = None
with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    filename = file.name
    file.write(supervisor.getUrdf().encode('utf-8'))
armChain = Chain.from_urdf_file(filename)

# Initialize the arm motors.
motors = []
for link in armChain.links:
    if 'Motor' in link.name:
        motor = supervisor.getMotor(link.name)
        motor.setVelocity(1.0)
        motors.append(motor)
        
def passiveWait(sec):
    start_time = supervisor.getTime();
    supervisor.step(timeStep);
    while start_time + sec > supervisor.getTime():
        supervisor.step(timeStep)   	

        
def openGripper(position=0.62):
  supervisor.getMotor('right_gripper').setPosition(position);
  supervisor.getMotor('left_gripper').setPosition(0 - position);
  passiveWait(2.0)

def closeGripper():
  supervisor.getMotor('right_gripper').setPosition(0.35);
  supervisor.getMotor('left_gripper').setPosition(-0.35);
  passiveWait(.5)
  
def move2pos(arm, x, y, z):
    armPosition = arm.getPosition()

    tx = x - armPosition[0]
    ty = y - armPosition[1]
    tz = z - armPosition[2]
    
    t = 1 if tz <= 0 else -1
    
    ikResults = armChain.inverse_kinematics([tx - 0.05, ty + 0.015, tz + t * 0.023])
    for i in range(len(motors)):
        motors[i].setPosition(ikResults[i + 1])
            
    passiveWait(4.0)
        
radar = supervisor.getRadar("radar")
radar.enable(timeStep)

# for i in range(5):
    # targetName = f'CUBE{i}'
    # print(targetName)
    # target = supervisor.getFromDef(targetName)
    # openGripper()
    # print('moveing')
    # move2pos(supervisor.getSelf(), *target.getPosition())
    # print("closing")
    # closeGripper()
    # target = supervisor.getFromDef('BASKET')
    # x, y, z = target.getPosition()
    # move2pos(supervisor.getSelf(), x, y + 0.3, z)
    # openGripper()
    
while supervisor.step(timeStep) != -1:
    targets = radar.getTargets()
    if targets:
        for target in targets:
            distance = target.distance
            print(distance)
            theta = target.azimuth
            x = distance * math.cos(theta)
            y = 0.66
            z = distance * math.sin(theta)
            openGripper()
            print('moveing')
            move2pos(supervisor.getSelf(), x + 0.99, y, z)
            print("closing")
            closeGripper()
            target = supervisor.getFromDef('BASKET')
            x, y, z = target.getPosition()
            move2pos(supervisor.getSelf(), x, y + 0.3, z)
            openGripper()
                
passiveWait(10)	
