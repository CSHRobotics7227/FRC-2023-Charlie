import wpilib
import wpimath.controller
import wpimath.trajectory
import commands2
import rev
import math

import const


class tiltSubsystem(commands2.ProfiledPIDSubsystem):
    def __init__(self) -> None:
        super().__init__(
            wpimath.controller.ProfiledPIDController(
                const.kTiltP,
                const.kTiltI,
                const.kTiltD,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    const.kTiltV,
                    const.kTiltA,
                ),
            ),
            const.TiltDefPos
        )
        self.tiltMotor = rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
        self.tiltMotor.setSmartCurrentLimit(30)
        self.encoder = wpilib.DutyCycleEncoder(1)
        self.disable()
        self.tiltMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

    def _useOutput(self, output: float, setpoint: float) -> None:
        self.tiltMotor.set(-output+.08)

    def _getMeasurement(self) -> float:
        position = self.encoder.getAbsolutePosition() * 10
        return position

    def periodic(self) -> None:
        super().periodic()
        if self.encoder.getAbsolutePosition() >= .9:
            self.tiltMotor.set(0)
        if self.encoder.getAbsolutePosition() <= .645:
            self.disable()
            self.tiltMotor.set(0)

    def cart2polar(self, x: float, y: float) -> None: # use for micro adjust
        self.enable()
        theta = math.atan(y / x)
        setpoint = ((theta / (-math.pi * 2)) + 0.743) * 10
        if setpoint>=9 or setpoint<=6.45: return
        self.setGoal(setpoint)

    def adjustTilt(self, add: float): # adjusts when called in loop
        self.enable()
        current = self.getController().getGoal().position
        setpoint = current+add
        if setpoint>=9 or setpoint<=6.45: return
        self.setGoal(setpoint)
