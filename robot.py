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

    #def autonomousPeriodic(self):
    #    self.container.robotDrive.goToDistance(50000)

    def teleopInit(self):
        self.container.enableAllPID()
        commands2.ScheduleCommand(extendArm(0, self.container.extender).andThen(tiltArm(8.52, self.container.tilter)))

    def teleopPeriodic(self) -> None:
        #self.setpoint += 0.02 * 0.02 * self.container.driverController.getRawAxis(1)
        #self.container.tilter.setGoal(self.setpoint)
        #print('setpoint = ', self.setpoint)
        #maxA = (self.container.driverController.getRawAxis(3)+1)/2
        #maxV = (self.container.driverController.getRawAxis(3)+1)/2
        #print('max A = ', maxA)
        #print('max V = ', maxV)
        #self.container.tilter.getController().setConstraints(trajectory.TrapezoidProfile.Constraints(
        #        0.75*4*maxV,
        #        0.9*4*maxA,
        #    ))
        #self.setpoint += 0.02*8*self.container.driverController2.getRawAxis(1)
        ##commands2.ScheduleCommand(move2cart(self.setpoint, -30, self.container.tilter, self.container.extender))
        #self.container.tilter.cart2polar(self.setpoint, -28)
        #self.container.extender.cart2polar(self.setpoint, -28)
        pass

    def testInit(self) -> None:
        pass

    def testPeriodic(self):
        pass

    def disabledInit(self):
        self.container.disableAllPID()


if __name__ == "__main__":
    wpilib.run(Robot)
