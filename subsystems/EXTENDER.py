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
        self.extendMotor = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless) # must give motor type, neos are brushless
        self.extendMotor.restoreFactoryDefaults() # for consistency's sake
        # NEO 550 CAN EASILY BE BURNT OUT, so its best practice to set a current limit
        self.extendMotor.setSmartCurrentLimit(30) # current limit of 30 amps
        self.extendMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.encoder = self.extendMotor.getEncoder() # You must assign self.extendMotor.getEncoder() to its own variable

        self.limitSwitch = wpilib.DigitalInput(0)
        self.disable() # disable PID controller at boot

    def _useOutput(self, output: float, setpoint: float) -> None:
        self.extendMotor.set(output)

    def _getMeasurement(self) -> float:
        return self.encoder.getPosition()

    def periodic(self) -> None:
        super().periodic() # be sure to call super, or PID controller won't work
        if not self.limitSwitch.get():
            self.encoder.setPosition(0) # zero arm when limit switch pressed

    def cart2polar(self, x: float, y: float): # use this function for micro adjust
        self.enable() # explanation of math in move2cart.py
        r = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
        extendSetpoint = (r - 37) * -2.4667
        self.setGoal(extendSetpoint)

    def adjustExtender(self, add: float): # when called in loop this will slowly change the set point
        self.enable()
        current = self.getController().getGoal().position
        setpoint = current+add
        if -setpoint<0 or -setpoint>=80: return
        self.setGoal(setpoint)
