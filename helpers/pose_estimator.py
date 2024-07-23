from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.kinematics import SwerveModulePosition
from constants import AutoConstants


class PoseEstimator:
    def __init__(self):
        super().__init__()
        self.pose = Pose2d()
        self.ready_to_update_pose = False

    def set_pose(self, pose: Pose2d) -> None:
        self.pose = pose

    def get_ready_to_update_pose(self) -> bool:
        return self.ready_to_update_pose
