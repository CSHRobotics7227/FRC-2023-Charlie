import wpilib
import commands2
import photonvision


class lockdown(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.camera = photonvision.PhotonCamera('USB_GS_Camera')
        self.results = self.camera.getLatestResult()

    def setPipeline(self, index: int) -> None:
        self.camera.setPipelineIndex(index)

    def getResults(self) -> photonvision.PhotonTrackedTarget:
        return self.results.getBestTarget()

    def periodic(self) -> None:
        self.results = self.camera.getLatestResult()

