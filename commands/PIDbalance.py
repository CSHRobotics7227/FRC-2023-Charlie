# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
import math
import wpilib
import commands2
import commands2.cmd
import wpimath.controller

from subsystems.DRIVE import DriveSubsystem

import const


class gyroBalance(commands2.PIDCommand):

    def __init__(self, drive: DriveSubsystem) -> None:
        super().__init__(
            wpimath.controller.PIDController(
                constants.kbalanceP,
                constants.kbalanceI,
                constants.kbalanceD,
            ),
            # Close loop on heading
            drive.tanHeading,
            # Set reference to target
            0,
            # Pipe output to turn robot
            lambda output: drive.arcadeDrive(-output, 0),
            # Require the drive
            [drive],
        )
        self.drive = drive

    #def initialize(self) -> None:
    #    super().initialize()
    #    self.drive.gyro.setYawAxis(wpilib.ADIS16470_IMU.IMUAxis.kX)

    #def execute(self) -> None:
    #    super().execute()
    #    print('balancing with output = ', )

    def end(self, interrupted: bool) -> None:
        super().end(interrupted)
        self.drive.gyro.setYawAxis(wpilib.ADIS16470_IMU.IMUAxis.kY)



    def isFinished(self) -> bool:
        print("GYRO balance = ", self.drive.tanHeading())
        return False #self.getController().atSetpoint()