""" from controller import Robot
from controller import Motor
from controller import DistanceSensor
from controller import Receiver
import struct
#create robot and rceiver instance
robot = Robot()
receiver = Receiver("receiver")
#TIME_STEP = int(robot.getBasicTimeStep())
TIME_STEP = 64
#COMM_CHANNEL = -1
SPEED = 6
COMMUNICATION_CHANNEL = 1
#isTooClose = False
#WbDeviceTag ds0, ds1, communication, left_motor, right_motor;
message_printed = 0
left_motor = robot.getMotor("left wheel motor")
right_motor = robot.getMotor("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
ds0 = robot.getDistanceSensor("ds0")
ds1 = robot.getDistanceSensor("ds1")
ds0.enable(TIME_STEP)
ds1.enable(TIME_STEP)
#communication = robot.getReceiver("receiver")
#receiver.enable(1)
#receiver.setChannel(COMMUNICATION_CHANNEL)
robot_type = 1
communication = robot.getReceiver("receiver")
Receiver.enable(communication, TIME_STEP)
    
    
while robot.step(TIME_STEP) != -1:
    if (Receiver.getQueueLength(communication) > 0):
        new_message= receiver.getData()
        message = struct.unpack("ddd", new_mssage)
        if (message_printed != 1) :
          # /* print null-terminated message */
          print(f"{message[0]} {message[1]} {message[2]}")
          message_printed = 1
            
        # /* fetch next packet */
        Receiver.nextPacket(communication)
    else :
        if (message_printed != 2) :
          print("Communication broken!\n")
          message_printed = 2
        #" new_message=receiver.getData()
        message = struct.unpack("ddd",new_message)
        #if (message_printed != 1):
        print(f"{message[0]} {message[1]} {message[2]}")
            #print("Communicating: received ",dataList,"\n")
        message_printed = 1
        receiver.nextPacket() "#
    #" else:
        if(message_printed != 2):
            print("Communication broken!\n")
            message_printed = 2 "#
            
    
    ds0_value = DistanceSensor.getValue(ds0)
    ds1_value = DistanceSensor.getValue(ds1)
    if(ds1_value > 500):
        if(ds0_value > 200):
            left_speed = -SPEED / 2
            right_speed = -SPEED
            #" left_speed = -SPEED
            right_speed = -SPEED/2 "#
        else:
         left_speed = -ds1_value/100
         right_speed = (ds0_value/100) + 0.5
    elif(ds0_value > 500):
        left_speed = (ds1_value / 100) + 0.5
        right_speed = -ds0_value / 100
    else:
        left_speed = SPEED
        right_speed = SPEED
        
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
 """
 
 """Receiver controller."""

# Import modules
from controller import Robot
from controller import Motor
from controller import DistanceSensor
from controller import Receiver
import struct

# Create robot instance
robot = Robot()

# Declare constansts
TIMESTEP = int(robot.getBasicTimeStep())
COMM_CHANNEL = -1
ROBOT_SPEED = 5.0

# Sensor setup

ds = []
dsNames = ['ds0', 'ds1']
for name in dsNames:
    ds.append(robot.getDistanceSensor(name))
for i in range(2):
    ds[i].enable(TIMESTEP)
isTooClose = False

# Motor setup

wheels = []
wheelsNames = ['left wheel motor', 'right wheel motor']
for name in wheelsNames:
    wheels.append(robot.getMotor(name))
for i in range(2):
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

# Receiver setup

receiver = robot.getReceiver('receiver')
receiver.enable(1)
receiver.setChannel(COMM_CHANNEL)

# Main loop:

while robot.step(TIMESTEP) != -1:

    # Just receiver stuff...
    
    if receiver.getQueueLength() > 0:
        newMessage = receiver.getData()
        message = struct.unpack("ddd", newMessage)
        print(f"{message[0]} {message[1]} {message[2]}")
        receiver.nextPacket()
        
    leftSpeed = ROBOT_SPEED
    rightSpeed = ROBOT_SPEED

    # Read sensors
    for i in range(2):
        if ds[i].getValue() > 500.0:
            leftSpeed += ROBOT_SPEED
            rightSpeed -= ROBOT_SPEED
            if ds[i].getValue() > 900:
                leftSpeed += ROBOT_SPEED
                rightSpeed -= ROBOT_SPEED
                if ds[i].getValue() > 950:
                    isTooClose = True
    if isTooClose == True:
        leftSpeed = 0
        rightSpeed = ROBOT_SPEED * -3.0
        isTooClose = False
    
    # Act on sensor data
    
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)