import math

import commands2.cmd

import const
from subsystems.DRIVE import DriveSubsystem

class setHeading(commands2.CommandBase):

    def __init__(self, inches: float, drive: DriveSubsystem) -> None:
        super().__init__()
        self.revolutions = -(inches / 18.85) * 8.85
        self.drive = drive

        self.addRequirements(drive)

    def initialize(self) -> None:
        super().initialize()
        self.drive.setBrake()
        self.drive.zeroHeading()
        self.drive.resetEncoders()

    def execute(self) -> None:
        super().execute()
        direction = (self.revolutions - self.drive.getAvgDistance()) / math.fabs((self.revolutions - self.drive.getAvgDistance()))
        gyroError = self.drive.getHeading()
        self.drive.arcadeDrive(const.headingSpeed*direction, gyroError*const.headingK)

    def end(self, interrupted: bool) -> None:
        super().end(interrupted)
        self.drive.arcadeDrive(0,0)
    def isFinished(self) -> bool:
        return math.fabs(self.drive.getAvgDistance() - self.revolutions) < 0.4