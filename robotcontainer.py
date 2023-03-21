import math
import wpilib

import commands2
import commands2.cmd
from commands2.button import CommandJoystick

from subsystems.DRIVE import DriveSubsystem
from subsystems.TILT import tiltSubsystem
from subsystems.EXTENDER import extenderSubsystem
from subsystems.CLAW import claw
from subsystems.LOCKDOWN import lockdown

from commands.move2cart import move2cart

from commands.gotoangle import TurnToAngle
from commands.goheading import goHeading
from commands.tiltArm import tiltArm
from commands.extendArm import extendArm


class RobotContainer:

    def __init__(self):
        self.imu = wpilib.ADIS16470_IMU()
        self.imu.calibrate()
        self.imu.setYawAxis(wpilib.ADIS16470_IMU.IMUAxis.kY)

        self.robotDrive = DriveSubsystem(self.imu)

        self.driverController = CommandJoystick(0)
        self.driverController2 = CommandJoystick(1)

        self.tilter = tiltSubsystem()
        self.extender = extenderSubsystem()

        self.claw = claw()
        self.lockdown = lockdown()

        self.configureButtonBindings()

        self.robotDrive.setDefaultCommand(
            commands2.RunCommand(
                lambda:  self.robotDrive.arcadeDrive(self.driverController.getRawAxis(1), self.driverController.getRawAxis(2), self.driverController.getRawButton(2), self.driverController.getRawButton(8), -(self.driverController.getRawAxis(3)+1)/2),
                [self.robotDrive],
            )
        )


    def configureButtonBindings(self):
        #self.driverController.button(1).onTrue(
        #    commands2.ScheduleCommand(commands.gotoangle.TurnToAngle(45, self.robotDrive))
        #)
        self.driverController.button(1).onTrue(
            commands2.cmd.runOnce(lambda:
                                  self.claw.toggleGrab(), [self.claw])
        )
        self.driverController.button(3).onTrue(
            commands2.cmd.runOnce(lambda:
                                  self.claw.tiltUp(), [self.claw])
        )
        self.driverController.button(4).onTrue(
            commands2.cmd.runOnce(lambda:
                                  self.claw.tiltDown(), [self.claw])
        )
        self.driverController.button(5).onTrue(
            commands2.cmd.runOnce(lambda:
                                  self.claw.release(), [self.claw])
        )
        self.driverController.button(6).onTrue(
            commands2.cmd.runOnce(lambda:
                                  self.claw.grip(), [self.claw])
        )
        self.driverController.button(10).onTrue(
            commands2.cmd.runOnce(lambda:
                                  self.robotDrive.setBrake(), [self.robotDrive])
        )
        self.driverController.button(10).onFalse(
            commands2.cmd.runOnce(lambda:
                                  self.robotDrive.setCoast(), [self.robotDrive])
        )
        self.driverController.button(11).onTrue(
            commands2.cmd.runOnce(lambda:
                                  self.lockdown.toggleLock(), [self.lockdown])
        )
        self.driverController2.button(7).onTrue(
            commands2.ScheduleCommand(move2cart(51+3, 3, self.tilter, self.extender))
        )
        self.driverController2.button(8).onTrue(
            commands2.ScheduleCommand(move2cart(66.5+5, 17+2, self.tilter, self.extender))
        )
        self.driverController2.button(9).onTrue(
            commands2.ScheduleCommand(move2cart(51+3, -5, self.tilter, self.extender))
        )
        self.driverController2.button(10).onTrue(
            commands2.ScheduleCommand(move2cart(66.5+5, 9, self.tilter, self.extender))
        )
        self.driverController2.button(11).onTrue(
            commands2.ScheduleCommand(move2cart(43, 2, self.tilter, self.extender))
        )
        self.driverController2.button(12).onTrue(
            commands2.ScheduleCommand(extendArm(0, self.extender).andThen(tiltArm(8.8, self.tilter)))
        )
        self.driverController2.button(1).onTrue(
            commands2.ScheduleCommand(extendArm(0, self.extender).andThen(tiltArm(8.52, self.tilter)))
        )



    def disableAllPID(self):
        self.tilter.disable()
        self.extender.disable()
    def enableAllPID(self):
        self.tilter.enable()
        self.extender.enable()

