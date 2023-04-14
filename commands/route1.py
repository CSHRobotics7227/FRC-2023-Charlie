import commands2.cmd
from commands2 import InstantCommand

from subsystems.DRIVE import DriveSubsystem
from subsystems.LOCKDOWN import lockdown
from subsystems.CLAW import claw
from subsystems.TILT import tiltSubsystem
from subsystems.EXTENDER import extenderSubsystem
from commands.tiltArm import tiltArm
from commands.extendArm import extendArm
from commands.setHeading import setHeading
from commands.bangBangBalance import gyroBalance2
from commands.move2cart import move2cart



class route1(commands2.SequentialCommandGroup):
    def __init__(self, tilter: tiltSubsystem, extender: extenderSubsystem, drive: DriveSubsystem, Lockdown: lockdown, Claw: claw):
        super().__init__()
        self.addRequirements(tilter, extender, drive, Lockdown, Claw)

        safety = extendArm(0, extender).andThen(tiltArm(8.83, tilter))
        safety.addRequirements(tilter, extender)

        self.addCommands(
            InstantCommand(lambda: Lockdown.lockUp(), [Lockdown]),
            InstantCommand(lambda: Claw.grip(), [Claw]),
            move2cart(69.5, 19, tilter, extender),
            commands2.WaitCommand(.1),
            InstantCommand(lambda: Claw.release(), [Claw]),
            commands2.WaitCommand(.2),
            safety,
            setHeading(-120, drive),
            commands2.WaitCommand(.5),
            setHeading(60, drive),
            gyroBalance2(drive, Lockdown),
        )

