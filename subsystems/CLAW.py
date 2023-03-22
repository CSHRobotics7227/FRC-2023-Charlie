import wpilib
import commands2


class claw(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.tiltCyl = wpilib.DoubleSolenoid(8, wpilib.PneumaticsModuleType.REVPH, 2, 3)
        self.clawCyl = wpilib.DoubleSolenoid(8, wpilib.PneumaticsModuleType.REVPH, 5, 4)

    def tiltDown(self):
        self.tiltCyl.set(wpilib.DoubleSolenoid.Value.kForward)
    def tiltUp(self):
        self.tiltCyl.set(wpilib.DoubleSolenoid.Value.kReverse)

    def grip(self):
        self.clawCyl.set(wpilib.DoubleSolenoid.Value.kForward)
    def release(self):
        self.clawCyl.set(wpilib.DoubleSolenoid.Value.kReverse)

    def toggleTilt(self):
        if self.tiltCyl.get() == wpilib.DoubleSolenoid.Value.kReverse:
            self.tiltCyl.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.tiltCyl.set(wpilib.DoubleSolenoid.Value.kReverse)

    def toggleGrab(self):
        if self.clawCyl.get() == wpilib.DoubleSolenoid.Value.kReverse:
            self.clawCyl.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.clawCyl.set(wpilib.DoubleSolenoid.Value.kReverse)
