import commands2
import wpilib
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
                    150,
                    300,
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
        drive.zeroHeading()
        #self.targetAngleDegrees = targetAngleDegrees
        self.getController().enableContinuousInput(-180, 180)
        # Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        # setpoint before it is considered as having reached the reference
        self.getController().setTolerance(1)
        drive.setBrake()

    #def initialize(self) -> None:
    #    super().initialize()
    #    self.__init__(self.targetAngleDegrees, self.drive)
    #    self.drive.zeroHeading()
    #    #self.getController().setGoal(self.targetAngleDegrees)

    #def execute(self) -> None:
    #    super().execute()
    #    print('setpoint = ', self.getController().getSetpoint().position)
    #    print('at setpoint = ', self.getController().atSetpoint())
    #    print('Turn Error = ', self.getController().getPositionError())


    #def execute(self) -> None:
    #    #print('heading = ', self.drive.getHeading())

    #def end(self, interrupted: bool) -> None:
    #    super().end(interrupted)
    #    self.drive.setCoast()

    def isFinished(self) -> bool:
        return self.getController().atGoal()
