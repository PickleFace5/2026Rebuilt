"""
VisionSubsystem!

Reads all cameras, updates camera inputs and throttles, etc.
"""
from math import inf
from typing import Callable

from commands2 import Subsystem
from ntcore import NetworkTableInstance
from phoenix6 import units
from pykit.logger import Logger
from wpilib import Alert
from wpimath.geometry import Pose2d

from constants import Constants
from subsystems.vision.io import VisionIO, ObservationType


# pylint: disable=too-few-public-methods, too-many-instance-attributes
class VisionSubsystem(Subsystem):
    """el visione"""


    def __init__(
        self,
        vision_consumer: Callable[[Pose2d, units.second, tuple[float, float, float] | None], None],
        *ios: VisionIO) -> None:
        super().__init__()

        self._consumer = vision_consumer
        self._io = ios

        # Initialize inputs
        self._inputs = tuple(VisionIO.VisionIOInputs() for _ in ios)

        # Initialize disconnected alerts
        self._alerts = tuple(
            Alert(f"Camera {cam.get_name()} is disconnected.", Alert.AlertType.kWarning)
            for cam in ios
        )

        # Pre-allocate lists and other things for fewer lookups
        self._tag_poses = [[] for _ in self._inputs]
        self._robot_poses = [[] for _ in self._inputs]
        self._robot_poses_accepted = [[] for _ in self._inputs]
        self._robot_poses_rejected = [[] for _ in self._inputs]

        self._all_tag_poses = []
        self._all_robot_poses = []
        self._all_robot_poses_accepted = []
        self._all_robot_poses_rejected = []

        self._field_layout = Constants.FIELD_LAYOUT
        self._field_length = self._field_layout.getFieldLength()
        self._field_width = self._field_layout.getFieldWidth()

        self._max_ambiguity = Constants.VisionConstants.max_ambiguity
        self._max_z_error = Constants.VisionConstants.max_z_error

        self._linear_std_baseline = Constants.VisionConstants.linear_std_dev_baseline
        self._angular_std_baseline = Constants.VisionConstants.angular_std_dev_baseline

    # pylint: disable=too-many-locals
    def periodic(self) -> None:
        """Log and send all observations."""
        inputs = self._inputs
        alerts = self._alerts

        all_tag_poses = self._all_tag_poses
        all_robot_poses = self._all_robot_poses
        all_robot_poses_accepted = self._all_robot_poses_accepted
        all_robot_poses_rejected = self._all_robot_poses_rejected

        NetworkTableInstance.getDefault().flush()
        for i, (cam, inp) in enumerate(zip(self._io, inputs)):
            cam.update_inputs(inp)
            Logger.processInputs(f"Vision/{inp.name}", inp)
            alerts[i].set(not inp.connected)

        all_tag_poses.clear()
        all_robot_poses.clear()
        all_robot_poses_accepted.clear()
        all_robot_poses_rejected.clear()

        field = self._field_layout
        max_amb = self._max_ambiguity
        max_z = self._max_z_error
        field_len = self._field_length
        field_wid = self._field_width
        lin_base = self._linear_std_baseline
        ang_base = self._angular_std_baseline
        consumer = self._consumer

        tag_poses_list = self._tag_poses
        robot_poses_list = self._robot_poses
        robot_poses_accepted_list = self._robot_poses_accepted
        robot_poses_rejected_list = self._robot_poses_rejected

        for i, camera in enumerate(inputs):
            cam_tag_poses = tag_poses_list[i]
            cam_robot_poses = robot_poses_list[i]
            cam_robot_poses_accepted = robot_poses_accepted_list[i]
            cam_robot_poses_rejected = robot_poses_rejected_list[i]
            cam_tag_poses.clear()
            cam_robot_poses.clear()
            cam_robot_poses_accepted.clear()
            cam_robot_poses_rejected.clear()

            for tag_id in camera.tag_ids:
                pose = field.getTagPose(tag_id)
                if pose is not None:
                    cam_tag_poses.append(pose)

            for observation in camera.observations:
                reject_pose = (
                        observation.tag_count == 0
                        or (
                                observation.tag_count == 1
                                and observation.ambiguity > max_amb
                        )
                        or abs(observation.pose.Z()) > max_z
                        or not (0.0 <= observation.pose.X() <= field_len)
                        or not (0.0 <= observation.pose.Y() <= field_wid)
                )

                cam_robot_poses.append(observation.pose)
                if reject_pose:
                    cam_robot_poses_rejected.append(observation.pose)
                else:
                    cam_robot_poses_accepted.append(observation.pose)

                if reject_pose:
                    continue

                std_dev_factor = (
                        (observation.avg_tag_dist * observation.avg_tag_dist)
                        / observation.tag_count
                )
                linear_std = lin_base * std_dev_factor
                if observation.observation == ObservationType.MEGATAG_2:
                    angular_std = inf
                else:
                    angular_std = ang_base * std_dev_factor

                consumer(
                    observation.pose.toPose2d(),
                    observation.timestamp,
                    (linear_std, linear_std, angular_std)
                )

            Logger.recordOutput(
                f"Vision/{camera.name}/TagPoses",
                cam_tag_poses
            )
            Logger.recordOutput(
                f"Vision/{camera.name}/RobotPoses",
                cam_robot_poses
            )
            Logger.recordOutput(
                f"Vision/{camera.name}/RobotPosesAccepted",
                cam_robot_poses_accepted
            )
            Logger.recordOutput(
                f"Vision/{camera.name}/RobotPosesRejected",
                cam_robot_poses_rejected
            )
            all_tag_poses.extend(cam_tag_poses)
            all_robot_poses.extend(cam_robot_poses)
            all_robot_poses_accepted.extend(cam_robot_poses_accepted)
            all_robot_poses_rejected.extend(cam_robot_poses_rejected)

        Logger.recordOutput("Vision/AllTagPoses", all_tag_poses)
        Logger.recordOutput("Vision/AllRobotPoses", all_robot_poses)
        Logger.recordOutput(
            "Vision/AllRobotPosesAccepted",
            all_robot_poses_accepted
        )
        Logger.recordOutput(
            "Vision/AllRobotPosesRejected",
            all_robot_poses_rejected
        )

    def set_throttle(self, throttle: int):
        """
        Sets the throttle for all Limelights.
        This is to prevent overheating.
        """
        for cam in self._io:
            cam.set_throttle(throttle)
