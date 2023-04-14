import math

import commands2.cmd

from subsystems.DRIVE import DriveSubsystem

class setHeading(commands2.CommandBase):

    def __init__(self, inches: float, drive: DriveSubsystem) -> None:
        super().__init__()
        self.revolutions = -(inches / 18.85) * 8.85
        self.drive = drive
        self.forward = 0.4
        self.k = -0.0714173

        self.addRequirements(drive)

    def initialize(self) -> None:
        super().initialize()
        self.drive.setBrake()
        self.drive.zeroHeading()
        self.drive.resetEncoders()

    def execute(self) -> None:
        super().execute()
        direction = (self.revolutions-self.drive.getEncoders())/math.fabs((self.revolutions-self.drive.getEncoders()))
        gyroError = self.drive.getHeading()
        self.drive.arcadeDrive(self.forward*direction, gyroError*self.k)
        print('gyro Error = ', gyroError)
        print('direction = ', direction)

    def end(self, interrupted: bool) -> None:
        super().end(interrupted)
        self.drive.arcadeDrive(0,0)
    def isFinished(self) -> bool:
        return math.fabs(self.drive.getEncoders()-self.revolutions) < 0.4