import math

import commands2.cmd

from subsystems.DRIVE import DriveSubsystem
from subsystems.LOCKDOWN import lockdown
import const
class gyroBalance2(commands2.CommandBase):

    def __init__(self, drive: DriveSubsystem, lock: lockdown) -> None:
        super().__init__()
        self.drive = drive
        self.lockdown = lock

    def execute(self) -> None:
        super().execute()
        if math.fabs(self.drive.getYcomp()) < 10:
            self.drive.arcadeDrive(0,0)
            if math.fabs(self.drive.getYcomp()) < 2:
                self.lockdown.lockDown()
            else:
                self.lockdown.lockUp()
        elif self.drive.getYcomp()>11:
            self.drive.arcadeDrive(const.balanceSpeed, 0)
        elif self.drive.getYcomp()<-11:
            self.drive.arcadeDrive(-const.balanceSpeed, 0)

    def isFinished(self) -> bool:
        return False 