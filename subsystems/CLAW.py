import wpilib
import commands2


class claw(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()                 # (can ID, Type, port1, port2)
        self.clawCyl = wpilib.DoubleSolenoid(8, wpilib.PneumaticsModuleType.REVPH, 5, 4)
        self.grip()

    def grip(self):
        self.clawCyl.set(wpilib.DoubleSolenoid.Value.kForward)
    def release(self):
        self.clawCyl.set(wpilib.DoubleSolenoid.Value.kReverse)

    def toggleGrab(self):
        if self.clawCyl.get() == wpilib.DoubleSolenoid.Value.kReverse:
            self.clawCyl.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.clawCyl.set(wpilib.DoubleSolenoid.Value.kReverse)
