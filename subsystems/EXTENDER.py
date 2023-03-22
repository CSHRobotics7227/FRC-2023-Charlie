import wpilib
import wpimath.controller
import wpimath.trajectory
import commands2
import rev
import math

import constants


class extenderSubsystem(commands2.ProfiledPIDSubsystem):
    def __init__(self) -> None:
        super().__init__(
            wpimath.controller.ProfiledPIDController(
                constants.kExtendP,
                constants.kExtendI,
                constants.kExtendD,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    120,
                    200,
                ),
            ),
            0
        )
        self.extendMotor = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
        self.extendMotor.setSmartCurrentLimit(30)
        self.extendMotor.restoreFactoryDefaults()
        self.extendMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.encoder = self.extendMotor.getEncoder()

        self.limitSwitch = wpilib.DigitalInput(0)
        self.disable()

    def _useOutput(self, output: float, setpoint: float) -> None:
        #if math.fabs(output)>1: output=output/math.fabs(output)
        #print('output  = ', output)
        self.extendMotor.set(output)

    def _getMeasurement(self) -> float:
        return self.encoder.getPosition()

    def periodic(self) -> None:
        super().periodic()
        #print('extender tolerance', self.getController().getPositionTolerance())

        #print('extender pos = ', self.encoder.getPosition())
        #print('extender stepoint = ', self.getController().getSetpoint().position)
        if not self.limitSwitch.get():
            self.encoder.setPosition(0)
            #self.extendMotor.set(0)

    def cart2polar(self, x: float, y: float):
        r = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
        #if 42 < r < 72: return
        extendSetpoint = (r - 42) * -2.4667
        self.setGoal(extendSetpoint)

    def adjustExtender(self, add: float):
        current = self.getController().getGoal().position
        setpoint = current+add
        if -setpoint<0 or -setpoint>=76: return
        #print('adjusting extender to = ', setpoint)
        self.setGoal(setpoint)
