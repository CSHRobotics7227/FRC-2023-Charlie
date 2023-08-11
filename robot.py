#!/usr/bin/env python3
# This comment ^^^ can tell computers how to run this file,

import wpilib
from commands2 import TimedCommandRobot # importing only one Class/Function from a module does not give a performace boost, only makes things neater
import robotcontainer

"""
Whenever you see 
    def function(param: float): 
That means the the parameter is of type float, This is best practice when working with robotpy
`"""

# Typical example uses TimedRobot, I use TimedCommandRobot
# because it is optimized for Command Based programming
# It calls commands2.CommandScheduler.getInstance().run() automatically [typically you call it in robotPeriodic()]
class Robot(TimedCommandRobot):

    def robotInit(self) -> None: # " -> None" means that this function will return a None value. I think this is best practice when working with robotpy
        self.rob = robotcontainer.RobotContainer() # just a normal python class
        self.rob.lights.set(False) # Turn off green light


    def autonomousInit(self) -> None:
        self.rob.autoInit()
    def teleopInit(self) -> None:
        self.rob.teleopInit()
    def teleopPeriodic(self) -> None:
        self.rob.teleopPeriodic()

    # run by running test mode in DS
    def testInit(self) -> None:
        pass
    def testPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        self.rob.disableAllPID() # disable PID controllers so robot won't run when disabled
        self.rob.setDefaultPos()
        self.rob.lights.set(False)


# if you run this file directly wpilib.run(Robot) will be run
# if you import this file, __name__ will not be main, so you won't run it twice if you import it
if __name__ == "__main__": wpilib.run(Robot)
