import math

import commands2.cmd
from commands2 import Command
from commands2 import InstantCommand

from subsystems.EXTENDER import extenderSubsystem
from subsystems.TILT import tiltSubsystem
from commands.tiltArm import tiltArm
from commands.extendArm import extendArm


class move2cart(commands2.SequentialCommandGroup):
    def __init__(self, x: float, y: float, tilter: tiltSubsystem, extender: extenderSubsystem):
        super().__init__()
        self.addRequirements(tilter, extender)

        theta = math.atan(y/x)
        r = math.sqrt(math.pow(x, 2)+math.pow(y, 2))
        if 42 < r < 72: self.cancel()
        if (-math.pi/3) < theta < (math.pi/6): self.cancel()
        extendSetpoint = (r-42)*-2.4667
        tiltSetpoint = ((theta/(-math.pi*2))+0.743)*10
        #print('extender setpoint = ', extendSetpoint)
        #print('tilt setpoint = ', tiltSetpoint)

        self.addCommands(
            #InstantCommand(lambda: tilter.setGoal(tiltSetpoint)),
            #InstantCommand(lambda: extender.setGoal(extendSetpoint))
            extendArm(0, extender),
            tiltArm(tiltSetpoint, tilter),
            extendArm(extendSetpoint, extender)
        )

    def getInterruptionBehavior(self) -> Command.InterruptionBehavior:
        return commands2.Command.InterruptionBehavior.kCancelIncoming

