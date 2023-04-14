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
                constants.kDriveP,
                constants.kDriveI,
                constants.kDriveD,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    15,
                    30,
                ),
            ),
            drive.getEncoders,
            revolutions,
            lambda output, setpoint: None, #drive.arcadeDrive(output, 0),
            [drive],
        )
        #drive.resetEncoders()
        drive.setBrake()
        self.drive = drive
        #self.controller = wpimath.controller.PIDController(.24,0,0)

    #def initialize(self) -> None:
    #    super().initialize()
    #    self.drive.resetEncoders()
    #    print('encoders = ', self.drive.getEncoders())


    def execute(self) -> None:
        super().execute()
        print('going forward')
        #self.getController().calculate()


    #def end(self, interrupted: bool) -> None:
    #    super().end(interrupted)
    #    print('encoders at end = ', self.drive.getEncoders())

    def isFinished(self) -> bool:
        return math.fabs(self.getController().getGoal().position)-math.fabs(self.drive.getEncoders()) <= 0.3
