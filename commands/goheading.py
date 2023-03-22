import commands2.cmd
import wpimath.controller
import wpimath.trajectory

from subsystems.DRIVE import DriveSubsystem

import constants


class goHeading(commands2.ProfiledPIDCommand):

    def __init__(self, inches: float, drive: DriveSubsystem) -> None:
        revolutions = (inches/18.85)*10.75
        print('revolutions = ', revolutions)
        super().__init__(
            wpimath.controller.ProfiledPIDController(
                constants.kDriveP,
                constants.kDriveI,
                constants.kDriveD,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    90,
                    180,
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
        self.getController().setTolerance(1)

    def execute(self) -> None:
        super().execute()
        print('at setpoint = ', self.getController().atSetpoint())
        print('Distance Error = ', self.getController().getPositionError())

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return self.getController().atSetpoint()
