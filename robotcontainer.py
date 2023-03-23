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

from commands.goheading import goHeading
from commands.gotoangle import TurnToAngle
from commands.tiltArm import tiltArm
from commands.extendArm import extendArm
from commands.route1 import route1


class RobotContainer:

    def __init__(self):
        self.imu = wpilib.ADIS16470_IMU()
        self.imu.calibrate()
        self.imu.setYawAxis(wpilib.ADIS16470_IMU.IMUAxis.kY)

        self.joy0 = CommandJoystick(0)
        self.joy1 = CommandJoystick(1)

        self.robotDrive = DriveSubsystem(self.imu)

        self.tilter = tiltSubsystem()
        self.extender = extenderSubsystem()

        self.claw = claw()
        self.lockdown = lockdown()

        self.groundPickup = extendArm(0, self.extender).andThen(tiltArm(8.8, self.tilter))
        self.groundPickup.addRequirements(self.tilter, self.extender)

        self.safety = extendArm(0, self.extender).andThen(tiltArm(8.52, self.tilter))
        self.safety.addRequirements(self.tilter, self.extender)
        self.configureButtonBindings()


        self.robotDrive.setDefaultCommand(
            commands2.RunCommand(
                lambda:
                self.robotDrive.arcadeDrive(self.joy0.getRawAxis(1), self.joy0.getRawAxis(2), self.joy0.getRawButton(2),
                                            self.joy0.getRawButton(8), -(self.joy0.getRawAxis(3) + 1) / 2),
                [self.robotDrive],
            )
        )


    def configureButtonBindings(self):
        self.joy0.button(1).onTrue(
            commands2.cmd.runOnce(lambda:
                self.claw.toggleGrab(), [self.claw])
        )
        self.joy0.button(3).onTrue(
            commands2.cmd.runOnce(lambda:
                self.claw.toggleTilt(), [self.claw])
        )

        self.joy0.button(4).onTrue(
            commands2.cmd.runOnce(lambda:
                commands2.CommandScheduler.getInstance().cancelAll(), [],
            )
        )
        self.joy0.button(10).onTrue(
            commands2.cmd.runOnce(lambda:
                self.robotDrive.setBrake(), [self.robotDrive])
        ).onFalse(
            commands2.cmd.runOnce(lambda:
                self.robotDrive.setCoast(), [self.robotDrive])
        )
        self.joy0.button(11).onTrue(
            commands2.cmd.runOnce(lambda:
                self.lockdown.toggleLock(), [self.lockdown])
        )
        #############
        ### JOY 1 ###
        #############
        self.joy1.button(7).onTrue(
            move2cart(54, 3, self.tilter, self.extender)
        ) # Mid Cone
        self.joy1.button(8).onTrue(
            move2cart(71.5, 19, self.tilter, self.extender)
        ) # High Cone
        self.joy1.button(9).onTrue(
            move2cart(54, -5, self.tilter, self.extender)
        ) # Mid Cube
        self.joy1.button(10).onTrue(
            move2cart(71.5, 9, self.tilter, self.extender)
        ) # High Cube
        self.joy1.button(11).onTrue(
            move2cart(43, 2, self.tilter, self.extender)
        ) # Loading Station
        self.joy1.button(12).onTrue(
            self.groundPickup
        ) # Ground Pickup
        self.joy1.button(1).onTrue(
            self.safety
        ) # Safety

    def autoCommand(self) -> commands2.Command:
        return route1(self.tilter, self.extender, self.robotDrive, self.lockdown, self.claw)

    def teleopPeriodic(self):
        if self.joy1.button(2):
            tiltAdd = self.joy1.getRawAxis(1)*0.02*0.1 # +/- 1 (joystick) * 1/50 (for total movement per sec) * (total encoder change per second)
            self.tilter.adjustTilt(tiltAdd)
            extendAdd = self.joy1.getRawAxis(0)*0.02*6 # +/- 1 (joystick) * 1/50 (for total movement per sec) * (total encoder change per second)
            self.extender.adjustExtender(extendAdd)

        if self.joy0.button(6):
            TurnToAngle(-45, self.robotDrive).schedule()
        if self.joy0.button(5):
            goHeading(-50, self.robotDrive).schedule()

        #commands2.CommandScheduler.getInstance().onCommandInterrupt(lambda: commands2.CommandScheduler.getInstance().cancelAll())

    def disableAllPID(self):
        self.tilter.disable()
        self.extender.disable()
    def enableAllPID(self):
        self.tilter.enable()
        self.extender.enable()

    def setDefaultPos(self):
        self.tilter.setGoal(8.52)
        self.extender.setGoal(0)

