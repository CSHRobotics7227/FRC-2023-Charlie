import math
import typing

import commands2.cmd
from commands2 import ProfiledPIDSubsystem
import wpimath.controller

from subsystems.EXTENDER import extenderSubsystem
from commands2 import Command
import constants


class extendArm(commands2.CommandBase):

    def __init__(self, target: float, extener: extenderSubsystem) -> None:
        super().__init__()
        self.target = target
        self.extender = extener
        #print('extend arm command created')

    def getInterruptionBehavior(self) -> Command.InterruptionBehavior:
        return commands2.Command.InterruptionBehavior.kCancelIncoming

    def initialize(self) -> None:
        super().initialize()
        self.extender.setGoal(self.target)
        #print('EXTENDING ARM')

    def isFinished(self) -> bool:
        #print('ARM EXTENDED = ', self.extender.getController().atSetpoint())
        #print('EXTENDER setpoint = ', self.extender.getController().getSetpoint())
        return math.fabs(self.extender.getController().getSetpoint().position-self.target) <= 0.9
