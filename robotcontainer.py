import time

import wpilib
import commands2
import commands2.cmd
from commands2.button import CommandJoystick

from subsystems.DRIVE import DriveSubsystem
from subsystems.TILT import tiltSubsystem
from subsystems.EXTENDER import extenderSubsystem
from subsystems.CLAW import claw
from subsystems.LOCKDOWN import lockdown
from subsystems.VISION import visionSubsystem

from commands.move2cart import move2cart
from commands.setAngle import setAngle
from commands.tiltArm import tiltArm
from commands.extendArm import extendArm
from commands.route1 import route1
from commands.setHeading import setHeading


class RobotContainer:

    def __init__(self):
        self.imu = wpilib.ADIS16470_IMU()
        self.imu.calibrate()
        self.imu.setYawAxis(wpilib.ADIS16470_IMU.IMUAxis.kY)

        self.joy0 = CommandJoystick(0)
        self.joy1 = CommandJoystick(1)

        self.drive = DriveSubsystem(self.imu)

        self.tilter = tiltSubsystem()
        self.extender = extenderSubsystem()

        self.claw = claw()
        self.lockdown = lockdown()

        self.vision = visionSubsystem()
        self.lights = wpilib.DigitalOutput(2)

        self.groundPickup = extendArm(0, self.extender).andThen(tiltArm(8.8, self.tilter))
        self.groundPickup.addRequirements(self.tilter, self.extender)

        self.safety = extendArm(0, self.extender).andThen(tiltArm(8.83, self.tilter))
        self.safety.addRequirements(self.tilter, self.extender)

        self.autoCommand = route1(self.tilter, self.extender, self.drive, self.lockdown, self.claw)

        self.lastTargetPress = time.time()

        self.configureButtonBindings()

        self.drive.setDefaultCommand(
            commands2.RunCommand(
                lambda:
                self.drive.arcadeDrive(
                    self.joy0.getRawAxis(1), self.joy0.getRawAxis(2), self.joy0.getRawButton(2),
                            self.joy0.getRawButton(8), -(-self.joy0.getRawAxis(3) + 1) / 2, self.joy0.getRawButton(7)),
                [self.drive],
            )
        )

    def configureButtonBindings(self):
        self.joy0.button(1).onTrue(
            commands2.cmd.runOnce(lambda:
                self.claw.toggleGrab(), [self.claw])
        ) # grip and release
        self.joy0.button(4).onTrue(
            commands2.cmd.runOnce(lambda:
                commands2.CommandScheduler.getInstance().cancelAll(), [],
            )
        ) # Cancel All
        self.joy0.button(9).onTrue(
            commands2.cmd.runOnce(lambda:
                self.toggleLight(), [],
            )
        ) # toggle light
        self.joy0.button(10).onTrue(
            commands2.cmd.runOnce(lambda:
                self.drive.setBrake(), [self.drive])
        ).onFalse(
            commands2.cmd.runOnce(lambda:
                self.drive.setCoast(), [self.drive])
        ) # brake hold down
        self.joy0.button(11).onTrue(
            commands2.cmd.runOnce(lambda:
                self.lockdown.toggleLock(), [self.lockdown])
        ) # toggle lockdown
        #############
        ### JOY 1 ###
        #############
        self.joy1.button(7).onTrue(
            move2cart(54, 3, self.tilter, self.extender)
        ) # Mid Cone
        self.joy1.button(8).onTrue(
            move2cart(69.5, 19, self.tilter, self.extender)
        ) # High Cone
        self.joy1.button(9).onTrue(
            move2cart(54, -5, self.tilter, self.extender)
        ) # Mid Cube
        self.joy1.button(10).onTrue(
            move2cart(69.5, 9, self.tilter, self.extender)
        ) # High Cube
        self.joy1.button(11).onTrue(
            move2cart(43, 3, self.tilter, self.extender)
        ) # Loading Station
        self.joy1.button(12).onTrue(
            self.groundPickup
        ) # Ground Pickup
        self.joy1.button(1).onTrue(
            self.safety
        ) # Safety
        self.joy1.button(4).onTrue(
            commands2.cmd.runOnce(lambda:
                commands2.CommandScheduler.getInstance().cancelAll(), [],
            )
        ) # Cancel all

    def autoInit(self):
        self.imu.setYawAxis(wpilib.ADIS16470_IMU.IMUAxis.kX)
        self.enableAllPID()
        self.drive.setBrake()
        self.autoCommand.schedule()

    def teleopInit(self):
        self.imu.setYawAxis(wpilib.ADIS16470_IMU.IMUAxis.kY)
        commands2.CommandScheduler.getInstance().cancelAll()
        self.enableAllPID()
        self.lockdown.lockUp()
        self.drive.setCoast()
        self.setDefaultPos()
        self.lights.set(True)

    def teleopPeriodic(self):
        #if self.joy1.button(2):
        #    tiltAdd = self.joy1.getRawAxis(1)*0.02*0.2 # +/- 1 (joystick) * 1/50 (for total movement per sec) * (total encoder change per second)
        #    self.tilter.adjustTilt(tiltAdd)
        #    extendAdd = self.joy1.getRawAxis(0)*0.02*6 # +/- 1 (joystick) * 1/50 (for total movement per sec) * (total encoder change per second)
        #    self.extender.adjustExtender(extendAdd)

        currentTime = time.time()

        if self.vision.apriltag.hasTargets() or self.vision.retro.hasTargets():
            if currentTime-self.lastTargetPress >= 0.33:
                if self.joy0.button(6):
                    distance2Target = self.vision.getAprilTargets().getBestCameraToTarget().x_feet * 12
                    setAngle(-self.vision.getAprilTargets().getYaw(), self.drive).andThen(setHeading(distance2Target - 39, self.drive)).schedule()
                    self.lastTargetPress = time.time()
                if self.joy0.button(5):
                    setAngle(-self.vision.getRetroTargets().getYaw(), self.drive).schedule()
                    self.lastTargetPress = time.time()

        if self.joy0.button(3) or self.joy1.button(3):
            self.enableAllPID()


    def disableAllPID(self):
        self.tilter.disable()
        self.extender.disable()
    def enableAllPID(self):
        self.tilter.enable()
        self.extender.enable()
    def setDefaultPos(self):
        self.tilter.setGoal(8.83)
        self.extender.setGoal(0)
    def toggleLight(self):
       if self.lights.get():
           self.lights.set(False)
       else:
           self.lights.set(True)
