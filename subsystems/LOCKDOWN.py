import wpilib
import commands2


class lockdown(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        print('creating cyl')
        self.lockDownCyl = wpilib.DoubleSolenoid(8, wpilib.PneumaticsModuleType.REVPH, 6, 7)

    def lockDown(self):
        self.lockDownCyl.set(wpilib.DoubleSolenoid.Value.kReverse)

    def lockUp(self):
        self.lockDownCyl.set(wpilib.DoubleSolenoid.Value.kForward)

    def toggleLock(self):
        if self.lockDownCyl.get() == wpilib.DoubleSolenoid.Value.kReverse:
            self.lockDownCyl.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.lockDownCyl.set(wpilib.DoubleSolenoid.Value.kReverse)