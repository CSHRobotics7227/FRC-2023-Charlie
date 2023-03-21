import wpilib
import wpimath.controller
import wpimath.trajectory
import commands2
import rev
import math

import constants


class tiltSubsystem(commands2.ProfiledPIDSubsystem):
    def __init__(self) -> None:
        super().__init__(
            wpimath.controller.ProfiledPIDController(
                constants.kTiltP,
                constants.kTiltI,
                constants.kTiltD,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    2.1,
                    3.0,
                ),
            )
        )
        self.tiltMotor = rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
        self.encoder = wpilib.DutyCycleEncoder(1)
        self.setGoal(7.5)
        self.disable()
        self.tiltMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

    def _useOutput(self, output: float, setpoint: float) -> None:
        #print('OUTPUT = ', -output+.07)
        self.tiltMotor.set(-output+.07)

    def _getMeasurement(self) -> float:
        #print('tilter pos = ', self.encoder.getAbsolutePosition())
        position = self.encoder.getAbsolutePosition() * 10
        return position

    def periodic(self) -> None:
        super().periodic()
        if self.encoder.getAbsolutePosition() >= .9:
            self.tiltMotor.set(0)
        if self.encoder.getAbsolutePosition() <= .645:
            self.disable()
            self.tiltMotor.set(0)
        #print('tilt arm', -2*math.pi*math.fmod(self.encoder.getAbsolutePosition()-0.74, 1))
        #print('tilt pos = ', self.encoder.getAbsolutePosition()*10)
        #print('tilt setpoint = ', self.getController().getSetpoint().position)
        #print('at setpoint = ', self.getController().atSetpoint())
        ##print('tilt error = ', self.getController().getPositionError())
        ##print('tilt tolerence = ', self.getController().getPositionTolerance())
        #print('is enabled = ', self.isEnabled())

    def cart2polar(self, x: float, y: float) -> None:
        theta = math.atan(y / x)
        #if (-math.pi/3) < theta < (math.pi/6): return
        tiltSetpoint = ((theta / (-math.pi * 2)) + 0.743) * 10
        self.setGoal(tiltSetpoint)