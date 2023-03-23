import math

import commands2.cmd
import wpimath.controller
import wpimath.trajectory

from subsystems.DRIVE import DriveSubsystem

import constants


class goHeading(commands2.ProfiledPIDCommand):
    def __init__(self, inches: float, drive: DriveSubsystem) -> None:
        revolutions = -(inches/18.85)*8.85
        print('revolutions = ', revolutions)
        super().__init__(
            wpimath.controller.ProfiledPIDController(
                constants.kDriveP,
                constants.kDriveI,
                constants.kDriveD,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    20,
                    40,
                ),
            ),
            # Close loop on heading
            drive.getEncoders,
            # Set reference to target
            revolutions,
            # Pipe output to turn robot
            lambda output, setpoint: drive.arcadeDrive(output, 0),
            # Require the drive
            [drive],
        )
        #self.getController().setTolerance()
        drive.resetEncoders()
        drive.setBrake()
        self.drive = drive


    def execute(self) -> None:
        super().execute()
        print('goal = ', math.fabs(self.getController().getGoal().position))
        print('encoders = ', math.fabs(self.drive.getEncoders()))

    def isFinished(self) -> bool:
        return math.fabs(self.getController().getGoal().position)-math.fabs(self.drive.getEncoders()) <= 0.3
