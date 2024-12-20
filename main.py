from tmc.TMC_2209_StepperDriver import *
import time
import random
import _thread


print("---")
print("SCRIPT START")
print("---")





#-----------------------------------------------------------------------
# initiate the TMC_2209 class
# use your pins for pin_step, pin_dir, pin_en here
#-----------------------------------------------------------------------
tmc = TMC_2209(27, 14, 26, 115200, 0)
tmc2 = TMC_2209(28, 15, 26, 115200, 0)





#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
tmc.setLoglevel(Loglevel.debug)
tmc.setMovementAbsRel(MovementAbsRel.relative)
tmc2.setLoglevel(Loglevel.debug)
tmc2.setMovementAbsRel(MovementAbsRel.relative)





#-----------------------------------------------------------------------
# these functions change settings in the TMC register
#-----------------------------------------------------------------------
tmc.setDirection_reg(False)
tmc.setVSense(True)
tmc.setCurrent(200)
tmc.setIScaleAnalog(True)
tmc.setInterpolation(True)
tmc.setSpreadCycle(False)
tmc.setMicrosteppingResolution(1)
tmc.setInternalRSense(False)

tmc2.setDirection_reg(False)
tmc2.setVSense(True)
tmc2.setCurrent(200)
tmc2.setIScaleAnalog(True)
tmc2.setInterpolation(True)
tmc2.setSpreadCycle(False)
tmc2.setMicrosteppingResolution(1)
tmc2.setInternalRSense(False)


print("---\n---")





#-----------------------------------------------------------------------
# these functions read and print the current settings in the TMC register
#-----------------------------------------------------------------------
#tmc.readIOIN()
#tmc.readCHOPCONF()
#tmc.readDRVSTATUS()
#tmc.readGCONF()

print("---\n---")





#-----------------------------------------------------------------------
# set the Accerleration and maximal Speed
#-----------------------------------------------------------------------
#tmc.setAcceleration(2000)
#tmc.setAcceleration(100)
tmc.setAcceleration(10)
tmc.setMaxSpeed(300)
#tmc.setMaxSpeed(30)

tmc2.setAcceleration(10)
tmc2.setMaxSpeed(300)




#-----------------------------------------------------------------------
# activate the motor current output
#-----------------------------------------------------------------------
tmc.setMotorEnabled(True)
tmc2.setMotorEnabled(True)




#-----------------------------------------------------------------------
# move the motor 1 revolution
#-----------------------------------------------------------------------
#tmc.runToPositionSteps(800)                             #move to position 400
#tmc.runToPositionSteps(0)                               #move to position 0
#tmc.doHoming(Direction.CW)

#count = 0

#try:
#    while True:
        #if count > 10:
        #    break
        
#        tmc.runToPositionSteps(200, MovementAbsRel.relative)    #move 400 steps forward
#        tmc.runToPositionSteps(-200, MovementAbsRel.relative)   #move 400 steps backward
        #count += 1
#except KeyboardInterrupt:
#    pass

movement_commands = []

with open('commands.txt', 'r') as file:
    for line in file:
        movement_commands.append(int(line.strip()))

print("Driving to positions:")
print(movement_commands)

def move1():
    while True:
        random_command_1 = random.choice(movement_commands)
        tmc.runToPositionSteps(random_command_1, MovementAbsRel.relative)
    
def move2():
    while True:
        random_command_1 = random.choice(movement_commands)
        tmc2.runToPositionSteps(random_command_1, MovementAbsRel.relative)

try:
    #pass
    #while True:
    _thread.start_new_thread(move2, ())
    move1()
    #move2()
    
        #move(tmc)
        #move(tmc2)
except KeyboardInterrupt:
    pass



#tmc.runToPositionSteps(400)
#tmc2.runToPositionSteps(400)                             #move to position 400
#tmc2.runToPositionSteps(0)                               #move to position 0





#-----------------------------------------------------------------------
# deactivate the motor current output
#-----------------------------------------------------------------------
tmc.setMotorEnabled(False)
tmc2.setMotorEnabled(False)

print("---\n---")





#-----------------------------------------------------------------------
# deinitiate the TMC_2209 class
#-----------------------------------------------------------------------
del tmc
del tmc2

print("---")
print("SCRIPT FINISHED")
print("---")
