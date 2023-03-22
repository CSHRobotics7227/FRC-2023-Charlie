import commands2.cmd
import wpimath.controller
import wpimath.trajectory

from subsystems.DRIVE import DriveSubsystem

import constants


class TurnToAngle(commands2.ProfiledPIDCommand):

    def __init__(self, targetAngleDegrees: float, drive: DriveSubsystem) -> None:
        super().__init__(
            wpimath.controller.ProfiledPIDController(
                constants.kTurnP,
                constants.kTurnI,
                constants.kTurnD,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    90,
                    180,
                ),
            ),
            # Close loop on heading
            drive.getHeading,
            # Set reference to target
            targetAngleDegrees,
            # Pipe output to turn robot
            lambda output, setpoint: drive.arcadeDrive(0, output),
            # Require the drive
            [drive],
        )
        self.getController().enableContinuousInput(-180, 180)
        # Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        # setpoint before it is considered as having reached the reference
        self.getController().setTolerance(1)
        print('target = ', targetAngleDegrees)

    def execute(self) -> None:
        super().execute()
        print('at setpoint = ', self.getController().atSetpoint())
        print('Turn Error = ', self.getController().getPositionError())


    #def execute(self) -> None:
    #    #print('heading = ', self.drive.getHeading())

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return self.getController().atSetpoint()
