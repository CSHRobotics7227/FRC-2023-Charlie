import commands2
import ctre
import math

import const


class DriveSubsystem(commands2.SubsystemBase):
    def __init__(self, gyro) -> None:
        super().__init__() # be sure to call super methods in all overrided methods. **especially periodic**. This allows subsystem base to function correctly
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

                 # (forward power    , turn power  ,half speed button, doublespeed     , turn adjust   , polarity )
    def arcadeDrive(self, power: float, turn: float, halfspeed=False, doublespeed=False, turn_power=0.3, reverse=False):
        if math.fabs(power) < const.powerDeadband: power = 0 # 0.0703125 is the deadbend
        if math.fabs(turn) < const.turnDeadband: turn = 0

        totalPower = .5
        if halfspeed: totalPower*=0.5

        if doublespeed:
            totalPower*=2
            turn_power*=0.5 # doublespeed shouldn't affect turn power.

        polarity = 1
        if reverse:
            polarity=-1
            turn_power*=-1

        # basic arcade drive formula is: {forward +/- turn} [+ or - depends on side]
        # add in turn power: {forward +/- (turn * turn_power)}
        # add in total power: {(forward +/- (turn * turn_power))*totalPower}
        # polarity will then reverse the direction, and turn_power is negated above, leading to reverse polarity
        self.LDrive1.set(ctre.ControlMode.PercentOutput, # PercentOutput is from -1 to 1. This is percent * max speed [configured above]
                         polarity*((power + (turn * turn_power)) * totalPower))
        self.RDrive2.set(ctre.ControlMode.PercentOutput,
                         polarity*((power - (turn * turn_power)) * totalPower))

    def resetEncoders(self):
        self.LDrive1.setSelectedSensorPosition(0) # zeros encoders
        self.RDrive2.setSelectedSensorPosition(0) # zeros encoders
    def zeroHeading(self):
        self.gyro.reset() # zeros gyro
    def getHeading(self):
        return self.gyro.getAngle()
    def getYcomp(self): # angle for charge station balance
        return self.gyro.getYComplementaryAngle()-87 # -87, so default angle is 0 degrees.
    def getAvgDistance(self): # getSelectedSensorPosition() returns how many encoder ticks have passed, each round has 2048, so this returns average number of rotations for both sides
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
