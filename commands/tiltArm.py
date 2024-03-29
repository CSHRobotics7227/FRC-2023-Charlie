# This is just to use setGoal() as a command
import math

import commands2.cmd
from commands2 import Command

from subsystems.TILT import tiltSubsystem

class tiltArm(commands2.CommandBase):

    def __init__(self, target: float, tilter: tiltSubsystem) -> None:
        super().__init__()
        self.target = target
        self.tilter = tilter

    def getInterruptionBehavior(self) -> Command.InterruptionBehavior:
        return commands2.Command.InterruptionBehavior.kCancelIncoming


    def initialize(self) -> None:
        super().initialize()
        self.tilter.enable()
        self.tilter.setGoal(self.target)


    def isFinished(self) -> bool:
        return math.fabs(self.tilter.getController().getSetpoint().position-self.target) <= 0.1