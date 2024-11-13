from wpimath.geometry import Pose2d


class DataWrap:
    def __init__(self):
        self.gp_in_view = False
        self.robot_pose = Pose2d(0, 0, 0)

    def set_gp_in_view(self, visible: bool) -> None:
        self.gp_in_view = visible

    def set_robot_pose(self, pose: Pose2d) -> None:
        self.robot_pose = pose
