import commands2

import wpimath.trajectory

from subsystems.DRIVE import DriveSubsystem


class goHeading(commands2.TrapezoidProfileCommand):
    def __init__(self, feet: float, drive: DriveSubsystem) -> None:
        super().__init__(
            wpimath.trajectory.TrapezoidProfile(
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    1, # max velocity
                    2, # max acceleration
                ),
                # End at desired position in meters; implicitly starts at 0
                wpimath.trajectory.TrapezoidProfile.State(feet, 0),
            ),
            lambda setpointState: drive.arcadeDrive(setpointState.velocity, 0),
            [drive], # require drive
        )
        drive.setBrake()
        drive.resetEncoders()