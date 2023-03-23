import math

import commands2.cmd
from commands2 import InstantCommand

from subsystems.DRIVE import DriveSubsystem
from subsystems.LOCKDOWN import lockdown
from subsystems.CLAW import claw
from subsystems.TILT import tiltSubsystem
from subsystems.EXTENDER import extenderSubsystem
from commands.tiltArm import tiltArm
from commands.extendArm import extendArm
from commands.goheading import goHeading
from commands.move2cart import move2cart



class route1(commands2.SequentialCommandGroup):
    def __init__(self, tilter: tiltSubsystem, extender: extenderSubsystem, drive: DriveSubsystem, Lockdown: lockdown, Claw: claw):
        super().__init__()
        self.addRequirements(tilter, extender, drive, Lockdown, Claw)

        safety = extendArm(0, extender).andThen(tiltArm(8.52, tilter))
        safety.addRequirements(tilter, extender)

        self.addCommands(
            InstantCommand(lambda: Claw.tiltUp(), [Claw]),
            InstantCommand(lambda: Claw.grip(), [Claw]),
            move2cart(73.5, 19, tilter, extender),
            commands2.WaitCommand(1),
            InstantCommand(lambda: Claw.release(), [Claw]),
            commands2.WaitCommand(.25),
            safety,
            goHeading(-75, drive),
            InstantCommand(lambda: Lockdown.lockDown(), [Lockdown],)
        )

