#!/usr/bin/env python3
from wpilib import run
from commands2 import TimedCommandRobot
import robotcontainer

class Robot(TimedCommandRobot):

    def robotInit(self) -> None:
        self.rob = robotcontainer.RobotContainer()
        self.rob.lights.set(False)

    def autonomousInit(self):
        self.rob.autoInit()

    def teleopInit(self):
        self.rob.teleopInit()
    def teleopPeriodic(self) -> None:
        self.rob.teleopPeriodic()

    def testInit(self) -> None:
        pass
    def testPeriodic(self):
        pass

    def disabledInit(self):
        self.rob.disableAllPID()
        self.rob.setDefaultPos()
        self.rob.lights.set(False)


if __name__ == "__main__": run(Robot)
