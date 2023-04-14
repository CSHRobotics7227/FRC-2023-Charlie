import math
import commands2.cmd
import wpimath.controller
import wpimath.trajectory

from subsystems.DRIVE import DriveSubsystem

import const


class setAngle(commands2.ProfiledPIDCommand):

    def __init__(self, targetAngleDegrees: float, drive: DriveSubsystem) -> None:
        super().__init__(
            wpimath.controller.ProfiledPIDController(
                const.kTurnP,
                const.kTurnI,
                const.kTurnD,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    const.kTurnV,
                    const.kTurnA,
                ),
            ),
            # Close loop on heading
            drive.getHeading,
            # Set reference to target
            targetAngleDegrees,
            # Pipe output to turn robot
            lambda output, setpoint: drive.arcadeDrive(0, output),
            [drive],
        )
        drive.zeroHeading()
        self.getController().enableContinuousInput(-180, 180)
        drive.setBrake()


    def isFinished(self) -> bool:
        return math.fabs(self.getController().getGoal().position)-math.fabs(self.drive.getHeading())<=1 and math.fabs(self.getController().getVelocityError())<12
