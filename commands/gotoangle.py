import math
import commands2.cmd
import wpimath.controller
import wpimath.trajectory

from subsystems.DRIVE import DriveSubsystem

import const


class TurnToAngle(commands2.ProfiledPIDCommand):

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
        self.drive = drive

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
        #print('velocy error = ', self.getController().getVelocityError())
        #print('pos error = ', self.getController().getPositionError())
        return math.fabs(self.getController().getGoal().position)-math.fabs(self.drive.getHeading())<=1 and math.fabs(self.getController().getVelocityError())<12
