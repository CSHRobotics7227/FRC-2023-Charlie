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
        self.imu = wpilib.ADIS16470_IMU() # name of gyro will change between different gyros
        self.imu.calibrate() # It's best to run calibrate every time the robot boots. If the robot is moving while this is called, it will mess up calibration
        self.imu.setYawAxis(wpilib.ADIS16470_IMU.IMUAxis.kY) # this sets the axis the getAngle() method will return.
                                                             # calling this will also zero the axis

        # joy0 is driver, joy1 is copilot
        self.joy0 = CommandJoystick(0) # there are many different types of Joysitcks in command based
        self.joy1 = CommandJoystick(1)

        self.robotDrive = DriveSubsystem(self.imu) # I'm passing in the gyro, so that I can access it globally, without having to pass in robotDrive

        self.tilter = tiltSubsystem()
        self.extender = extenderSubsystem()

        self.claw = claw()
        self.lockdown = lockdown()

        self.vision = visionSubsystem()
        self.lights = wpilib.DigitalOutput(2)

        # .andThen() runs a command after one command has finished, as a sequential command group.
        self.groundDropoff = extendArm(0, self.extender).andThen(tiltArm(8.5, self.tilter)).andThen(extendArm(-26.6, self.extender))
        # .andThen() doesn't automatically add requirements, and thus I call it here.
        # without this addRequirements, it was possible for extend arm to not be canceled, but tiltArm to be canceled, this caused weird behavior, like the arm slamming into the ground, and commands being stuck.
        self.groundDropoff.addRequirements(self.tilter, self.extender)

        self.safety = extendArm(0, self.extender).andThen(tiltArm(8.83, self.tilter))
        self.safety.addRequirements(self.tilter, self.extender)

        self.autoCommand = route1(self.tilter, self.extender, self.robotDrive, self.lockdown, self.claw)

        self.lastTargetPress = time.time() # last press time for debouncer for targets.

        self.configureButtonBindings() # defined below

        self.robotDrive.setDefaultCommand(
            commands2.RunCommand( # Runs command on loop. [a runnable is a lambda]
                lambda:
                self.robotDrive.arcadeDrive(
                    self.joy0.getRawAxis(1), self.joy0.getRawAxis(2), self.joy0.getRawButton(2),
                            self.joy0.getRawButton(8), -(-self.joy0.getRawAxis(3) + 1) / 2),
                [self.robotDrive], # second parameter requires robotDrive
            )
        )

    def configureButtonBindings(self):
        self.joy0.button(1).onTrue( # .onTrue() takes a command to run when button is pressed
    (lambda: # commands2.cmd.runOnce takes a lambda [so basically a function] and creates a command
                self.claw.toggleGrab(), [self.claw]) # second parameter is to require the claw in the command
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
                self.robotDrive.setBrake(), [self.robotDrive])
        ).onFalse( # on false will be called once, when the button is false
            commands2.cmd.runOnce(lambda:
                self.robotDrive.setCoast(), [self.robotDrive])
        ) # brake hold down
        self.joy0.button(11).onTrue(
            commands2.cmd.runOnce(lambda:
                self.lockdown.toggleLock(), [self.lockdown])
        ) # toggle lockdown
        #############
        ### JOY 1 ###
        #############
        self.joy1.button(7).onTrue(
            move2cart(55, 3, self.tilter, self.extender)
        ) # Mid Cone
        self.joy1.button(8).onTrue(
            move2cart(71, 19, self.tilter, self.extender)
        ) # High Cone
        self.joy1.button(9).onTrue(
            move2cart(55, -5, self.tilter, self.extender)
        ) # Mid Cube
        self.joy1.button(10).onTrue(
            move2cart(71, 9, self.tilter, self.extender)
        ) # High Cube
        self.joy1.button(11).onTrue(
            move2cart(43, 2, self.tilter, self.extender)
        ) # Loading Station
        self.joy1.button(5).onTrue(
            move2cart(43, 1, self.tilter, self.extender)
        ) # Loading Station
        self.joy1.button(6).onTrue(
            move2cart(43, 0, self.tilter, self.extender)
        ) # Loading Station
        self.joy1.button(12).onTrue(
            self.groundDropoff
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
        self.enableAllPID()
        self.robotDrive.setBrake() # motors should be in brake mode, so it is easier to have fine control
        self.autoCommand.schedule() # run command

    def teleopInit(self):
        commands2.CommandScheduler.getInstance().cancelAll() # ensure auto command is canceled
        self.enableAllPID()
        self.lockdown.lockUp()
        self.robotDrive.setCoast() # drivers like coast mode
        self.setDefaultPos() # Default Position is safety
        self.lights.set(True) # Turn lights on

    def teleopPeriodic(self):
        """This used to handle micro adjust, but it can extend out of frame perimeter so far"""
        #if self.joy1.button(2):
        #    tiltAdd = self.joy1.getRawAxis(1)*0.02*0.2 # +/- 1 (joystick) * 1/50 (for total movement per sec) * (total encoder change per second)
        #    self.tilter.adjustTilt(tiltAdd)
        #    extendAdd = self.joy1.getRawAxis(0)*0.02*6 # +/- 1 (joystick) * 1/50 (for total movement per sec) * (total encoder change per second)
        #    self.extender.adjustExtender(extendAdd)

        currentTime = time.time() # current time for debouncer
        if self.vision.apriltag.hasTargets() or self.vision.retro.hasTargets():
            if currentTime-self.lastTargetPress >= 0.33:
                try: # code will crash if setAngle called without target.
                    if self.joy0.button(6):
                        distance2Target = self.vision.getAprilTargets().getBestCameraToTarget().x_feet * 12 # you want the camera 39 inches from the target
                        setAngle(-self.vision.getAprilTargets().getYaw(), self.robotDrive).andThen(setHeading(distance2Target - 39, self.robotDrive)).schedule()
                        self.lastTargetPress = time.time()
                    if self.joy0.button(5):
                        setAngle(-self.vision.getRetroTargets().getYaw(), self.robotDrive).schedule()
                        self.lastTargetPress = time.time()
                except:
                    print('no targets')

        if self.joy0.button(3) or self.joy1.button(3): # IDK why this isn't bound to buttons normally
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
