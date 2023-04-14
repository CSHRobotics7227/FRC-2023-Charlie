import commands2
import ctre
import math

import const


class DriveSubsystem(commands2.SubsystemBase):
    def __init__(self, gyro) -> None:
        super().__init__()
        self.LDrive1 = ctre.TalonFX(1)
        self.RDrive2 = ctre.TalonFX(2)
        self.LDrive3 = ctre.TalonFX(3)
        self.RDrive4 = ctre.TalonFX(4)

        self.LDrive1.configFactoryDefault(30)  # wait 30 miliseconds to confirm
        self.LDrive3.configFactoryDefault(30)
        self.RDrive2.configFactoryDefault(30)
        self.RDrive4.configFactoryDefault(30)

        ### SET MAXIMIUM SPEED ###
        self.RDrive2.configPeakOutputForward(const.maxSpeed, 30)
        self.RDrive2.configPeakOutputReverse(-const.maxSpeed, 30)
        self.RDrive4.configPeakOutputForward(const.maxSpeed, 30)
        self.RDrive4.configPeakOutputReverse(-const.maxSpeed, 30)
        self.LDrive1.configPeakOutputForward(const.maxSpeed, 30)
        self.LDrive1.configPeakOutputReverse(-const.maxSpeed, 30)
        self.LDrive3.configPeakOutputForward(const.maxSpeed, 30)
        self.LDrive3.configPeakOutputReverse(-const.maxSpeed, 30)

        # zero encoders
        self.LDrive3.setSelectedSensorPosition(0)
        self.LDrive1.setSelectedSensorPosition(0)
        self.RDrive4.setSelectedSensorPosition(0)
        self.RDrive2.setSelectedSensorPosition(0)

        # ensure the 2 drive motors are going in the correct direction
        self.LDrive1.setInverted(True) # motors should be inverted in gearbox
        self.LDrive3.setInverted(True) # motors should be inverted in gearbox
        self.LDrive3.follow(self.LDrive1) # follow motor, so we only need to control 1
        self.RDrive4.follow(self.RDrive2) # follow motor, so we only need to control 1

        self.gyro = gyro

        self.gyro.reset() # set angle = 0

    def arcadeDrive(self, power: float, turn: float, halfspeed=False, doublespeed=False, turn_power=0.3, reverse=False):
        if math.fabs(power) < const.powerDeadband: power = 0 # 0.0703125 is the deadbend
        if math.fabs(turn) < const.turnDeadband: turn = 0

        #power-=const.powerDeadband
        #turn-=const.turnDeadband

        motor_power = .5
        if halfspeed: motor_power*=0.5
        if doublespeed:
            motor_power*=2
            turn_power*=0.5

        polarity = 1
        if reverse:
            polarity=-1
            turn_power*=-1

        self.LDrive1.set(ctre.ControlMode.PercentOutput,
                         polarity*((power + (turn * turn_power)) * motor_power))
        self.RDrive2.set(ctre.ControlMode.PercentOutput,
                         polarity*((power - (turn * turn_power)) * motor_power))

    def resetEncoders(self):
        self.LDrive1.setSelectedSensorPosition(0)
        self.RDrive2.setSelectedSensorPosition(0)
    def zeroHeading(self):
        self.gyro.reset()
    def getHeading(self):
        return self.gyro.getAngle()
    def tanHeading(self):
        return math.tan(self.gyro.getAngle())
    def getAvgDistance(self):
        return (self.LDrive1.getSelectedSensorPosition()/2048 + self.RDrive2.getSelectedSensorPosition()/2048)/2
    def setBrake(self):
        self.LDrive1.setNeutralMode(ctre.NeutralMode.Brake)
        self.LDrive3.setNeutralMode(ctre.NeutralMode.Brake)
        self.RDrive2.setNeutralMode(ctre.NeutralMode.Brake)
        self.RDrive4.setNeutralMode(ctre.NeutralMode.Brake)
    def setCoast(self):
        self.LDrive1.setNeutralMode(ctre.NeutralMode.Coast)
        self.LDrive3.setNeutralMode(ctre.NeutralMode.Coast)
        self.RDrive2.setNeutralMode(ctre.NeutralMode.Coast)
        self.RDrive4.setNeutralMode(ctre.NeutralMode.Coast)
