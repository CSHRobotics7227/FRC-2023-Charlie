import wpilib
import wpimath.controller
import wpimath.trajectory
import commands2
import rev
import math

import const


class extenderSubsystem(commands2.ProfiledPIDSubsystem):
    def __init__(self) -> None:
        super().__init__(
            wpimath.controller.ProfiledPIDController(
                const.kExtendP,
                const.kExtendI,
                const.kExtendD,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    const.kExtendV,
                    const.kExtendA,
                ),
            ),
            0
        )
        self.extendMotor.restoreFactoryDefaults()
        self.extendMotor = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
        self.extendMotor.setSmartCurrentLimit(30)
        self.extendMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.encoder = self.extendMotor.getEncoder()

        self.limitSwitch = wpilib.DigitalInput(0)
        self.disable()

    def _useOutput(self, output: float, setpoint: float) -> None:
        self.extendMotor.set(output)

    def _getMeasurement(self) -> float:
        return self.encoder.getPosition()

    def periodic(self) -> None:
        super().periodic()
        if not self.limitSwitch.get():
            self.encoder.setPosition(0)

    def cart2polar(self, x: float, y: float):
        self.enable()
        r = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
        extendSetpoint = (r - 37) * -2.4667
        self.setGoal(extendSetpoint)

    def adjustExtender(self, add: float):
        self.enable()
        current = self.getController().getGoal().position
        setpoint = current+add
        if -setpoint<0 or -setpoint>=80: return
        self.setGoal(setpoint)
