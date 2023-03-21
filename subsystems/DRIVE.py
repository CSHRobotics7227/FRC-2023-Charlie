import commands2
import ctre
import math

import constants


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
        self.RDrive2.configPeakOutputForward(constants.maxSpeed, 30)  # set speed to 35% and wait 30 ms
        self.RDrive2.configPeakOutputReverse(-constants.maxSpeed, 30)
        self.RDrive4.configPeakOutputForward(constants.maxSpeed, 30)
        self.RDrive4.configPeakOutputReverse(-constants.maxSpeed, 30)
        self.LDrive1.configPeakOutputForward(constants.maxSpeed, 30)
        self.LDrive1.configPeakOutputReverse(-constants.maxSpeed, 30)
        self.LDrive3.configPeakOutputForward(constants.maxSpeed, 30)
        self.LDrive3.configPeakOutputReverse(-constants.maxSpeed, 30)

        # zero encoders
        self.LDrive3.setSelectedSensorPosition(0)
        self.LDrive1.setSelectedSensorPosition(0)
        self.RDrive4.setSelectedSensorPosition(0)
        self.RDrive2.setSelectedSensorPosition(0)

        self.LDrive3.config_kP(0, 0.4)
        self.LDrive1.config_kP(0, 0.4)
        self.RDrive4.config_kP(0, 0.4)
        self.RDrive2.config_kP(0, 0.4)

        self.LDrive3.config_kF(0, 0.2)
        self.LDrive1.config_kF(0, 0.2)
        self.RDrive4.config_kF(0, 0.2)
        self.RDrive2.config_kF(0, 0.2)

        self.LDrive3.configMotionAcceleration(6000)
        self.LDrive1.configMotionAcceleration(6000)
        self.RDrive4.configMotionAcceleration(6000)
        self.RDrive2.configMotionAcceleration(6000)

        self.LDrive3.configMotionCruiseVelocity(15000)
        self.LDrive1.configMotionCruiseVelocity(15000)
        self.RDrive4.configMotionCruiseVelocity(15000)
        self.RDrive2.configMotionCruiseVelocity(15000)


        # ensure the 2 drive motors are going in the correct direction
        self.LDrive1.setInverted(True) # motors should be inverted in gearbox
        self.LDrive3.setInverted(True) # motors should be inverted in gearbox
        self.LDrive3.follow(self.LDrive1) # follow motor, so we only need to control 1
        self.RDrive4.follow(self.RDrive2) # follow motor, so we only need to control 1

        self.gyro = gyro
        self.gyro.reset() # set angle = 0

    def resetEncoders(self):
        self.LDrive1.setSelectedSensorPosition(0)
        self.RDrive2.setSelectedSensorPosition(0)

    def zeroHeading(self):
        self.gyro.reset()

    def getHeading(self):
        print('heading  = ', self.gyro.getAngle())
        return self.gyro.getAngle()

    def getEncoders(self):
        return self.LDrive1.getSelectedSensorPosition(), self.RDrive2.getSelectedSensorPosition()

    def getTurnRate(self):
        return self.gyro.getRate() * -1

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

    def arcadeDrive(self, power: float, turn: float, halfspeed=False, doublespeed=False, turn_power=0.3):
        if math.fabs(power) < 0.0703125: power = 0 # 0.0703125 is the deadbend
        if math.fabs(turn) < 0.0703125: turn = 0

        motor_power = .5  # (self.joystick2.getRawAxis(3)+1) / 2
        #print('setting motors = ', turn, power)
        if halfspeed: motor_power*=0.5
        if doublespeed:
            motor_power*=2
            turn_power*=0.5

        self.LDrive1.set(ctre.ControlMode.PercentOutput, ((power + (turn * turn_power)) * motor_power))
        self.RDrive2.set(ctre.ControlMode.PercentOutput, ((power - (turn * turn_power)) * motor_power))

    def goToDistance(self, distance: float):
        self.LDrive1.set(ctre.ControlMode.MotionMagic, distance)
        self.RDrive2.set(ctre.ControlMode.MotionMagic, distance)

    #def periodic(self) -> None:
    #    print('GYRO = ', self.gyro.getAngle())