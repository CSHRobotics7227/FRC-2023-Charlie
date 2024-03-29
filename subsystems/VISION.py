import commands2
import photonvision

class visionSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.apriltag = photonvision.PhotonCamera('USB_GS_Camera') # name must match what is on photon vision dashboard at http://10.90.68.11:5800
        self.retro = photonvision.PhotonCamera('USB_Color_Camera')
        self.aprilResults = self.apriltag.getLatestResult()
        self.retroResults = self.retro.getLatestResult()

    def setPipeline(self, index: int) -> None:
        self.apriltag.setPipelineIndex(index)

    def getRetroTargets(self) -> photonvision.PhotonTrackedTarget:
        return self.retroResults.getBestTarget() # returns the closest target
    def getAprilTargets(self) -> photonvision.PhotonTrackedTarget:
        return self.aprilResults.getBestTarget()


    def periodic(self) -> None:
        self.aprilResults = self.apriltag.getLatestResult()
        self.retroResults = self.retro.getLatestResult()
