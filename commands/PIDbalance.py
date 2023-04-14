import wpilib
import commands2.cmd
import wpimath.controller

from subsystems.DRIVE import DriveSubsystem

import const


class gyroBalance(commands2.PIDCommand):

    def __init__(self, drive: DriveSubsystem) -> None:
        super().__init__(
            wpimath.controller.PIDController(
                const.kbalanceP,
                const.kbalanceI,
                const.kbalanceD,
            ),
            # Close loop on heading
            drive.tanHeading,
            # Set reference to target
            0,
            # Pipe output to turn robot
            lambda output: drive.arcadeDrive(-output, 0),
            [drive],
        )
        self.drive = drive

    def end(self, interrupted: bool) -> None:
        super().end(interrupted)
        self.drive.gyro.setYawAxis(wpilib.ADIS16470_IMU.IMUAxis.kY)



    def isFinished(self) -> bool:
        return False #self.getController().atSetpoint()