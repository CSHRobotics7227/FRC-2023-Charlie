import math

import commands2.cmd
from commands2 import Command

from subsystems.TILT import tiltSubsystem

class tiltArm(commands2.CommandBase):

    def __init__(self, target: float, tilter: tiltSubsystem) -> None:
        super().__init__()
        self.target = target
        self.tilter = tilter
        #print('tilt arm command created')

    def getInterruptionBehavior(self) -> Command.InterruptionBehavior:
        return commands2.Command.InterruptionBehavior.kCancelIncoming


    def initialize(self) -> None:
        super().initialize()
        self.tilter.enable()
        self.tilter.setGoal(self.target)

    def isFinished(self) -> bool:
        #print('ARM TIlTED = ', self.tilter.getController().atSetpoint())
        #print('ARM setpoint = ', self.tilter.getController().getSetpoint())
        return math.fabs(self.tilter.getController().getSetpoint().position-self.target) <= 0.1