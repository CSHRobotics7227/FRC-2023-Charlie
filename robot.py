#!/usr/bin/env python3
import wpilib
import commands2
import commands2.cmd
import robotcontainer

from commands.extendArm import extendArm
from commands.tiltArm import tiltArm
class Robot(commands2.TimedCommandRobot):

    def robotInit(self) -> None:
        self.container = robotcontainer.RobotContainer()
        #self.setpoint = 42

    def autonomousInit(self):
        self.container.enableAllPID()
        self.container.robotDrive.setBrake()
        self.container.autoCommand().schedule()

    def teleopInit(self):
        self.container.enableAllPID()
        self.container.robotDrive.setCoast()
        extendArm(0, self.container.extender).andThen(tiltArm(8.52, self.container.tilter)).schedule()

    def teleopPeriodic(self) -> None:
        self.container.teleopPeriodic()

    def testInit(self) -> None:
        pass

    def testPeriodic(self):
        pass

    def disabledInit(self):
        self.container.disableAllPID()
        self.container.setDefaultPos()


if __name__ == "__main__":
    wpilib.run(Robot)
