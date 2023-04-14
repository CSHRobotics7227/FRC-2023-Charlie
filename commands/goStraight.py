import math

import commands2.cmd
import wpimath.controller
import wpimath.trajectory

from subsystems.DRIVE import DriveSubsystem

import const


class goStriaght(commands2.ProfiledPIDCommand):
    def __init__(self, inches: float, drive: DriveSubsystem) -> None:
        revolutions = -(inches/18.85)*8.85
        super().__init__(
            wpimath.controller.ProfiledPIDController(
                const.kDriveForwardP,
                const.kDriveForwardI,
                const.kDriveForwardD,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    const.kDriveForwardV,
                    const.kDriveForwardA,
                ),
            ),
            drive.getAvgDistance,
            revolutions,
            lambda output, setpoint: None, #drive.arcadeDrive(output, 0),
            [drive],
        )
        drive.setBrake()
        self.drive = drive


    def isFinished(self) -> bool:
        return math.fabs(self.getController().getGoal().position)-math.fabs(self.drive.getAvgDistance()) <= 0.3
