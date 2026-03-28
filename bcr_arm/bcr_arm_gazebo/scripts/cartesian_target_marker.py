#!/usr/bin/env python3

import subprocess

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class CartesianTargetMarker(Node):
    """Create and move a visible Gazebo marker using Gazebo's native service transport."""

    def __init__(self):
        super().__init__("cartesian_target_marker")

        self.declare_parameter("target_topic", "/cartesian_target")
        self.declare_parameter("world_name", "default")
        self.declare_parameter("marker_name", "cartesian_target_marker")
        self.declare_parameter("marker_radius", 0.05)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("z_offset", 0.0)
        self.declare_parameter("timeout_ms", 3000)

        self._target_topic = self.get_parameter("target_topic").value
        self._world_name = self.get_parameter("world_name").value
        self._marker_name = self.get_parameter("marker_name").value
        self._marker_radius = float(self.get_parameter("marker_radius").value)
        self._frame_id = self.get_parameter("frame_id").value
        self._z_offset = float(self.get_parameter("z_offset").value)
        self._timeout_ms = int(self.get_parameter("timeout_ms").value)

        self._spawned = False

        self._target_sub = self.create_subscription(
            PointStamped, self._target_topic, self._target_callback, 10
        )

        self.get_logger().info(
            "Listening on %s and updating Gazebo marker '%s' in world '%s'"
            % (self._target_topic, self._marker_name, self._world_name)
        )

    def _target_callback(self, msg: PointStamped) -> None:
        frame_id = msg.header.frame_id or self._frame_id
        if frame_id != self._frame_id:
            self.get_logger().warning(
                "Ignoring target in frame '%s'; expected '%s'." % (frame_id, self._frame_id)
            )
            return

        x_pos = float(msg.point.x)
        y_pos = float(msg.point.y)
        z_pos = float(msg.point.z) + self._z_offset
        self.get_logger().info(
            "Received target x=%.3f y=%.3f z=%.3f frame=%s"
            % (x_pos, y_pos, z_pos, frame_id)
        )

        if not self._spawned:
            if self._spawn_marker(x_pos, y_pos, z_pos):
                self._spawned = True
                self.get_logger().info("Spawned Gazebo marker.")
                return

            # The marker may already exist from a previous run. Try moving it.
            if self._move_marker(x_pos, y_pos, z_pos):
                self._spawned = True
                self.get_logger().info("Marker already existed; moved it instead.")
                return

            self.get_logger().error("Failed to spawn or move Gazebo marker.")
            return

        if self._move_marker(x_pos, y_pos, z_pos):
            self.get_logger().info(
                "Moved Gazebo marker to x=%.3f y=%.3f z=%.3f" % (x_pos, y_pos, z_pos)
            )
        else:
            self.get_logger().error("Failed to move Gazebo marker.")

    def _spawn_marker(self, x_pos: float, y_pos: float, z_pos: float) -> bool:
        service = f"/world/{self._world_name}/create"
        sdf = self._build_marker_sdf()
        request = (
            f'name: "{self._marker_name}" '
            f'allow_renaming: false '
            f'sdf: "{sdf}" '
            f'pose {{ position {{ x: {x_pos} y: {y_pos} z: {z_pos} }} '
            f'orientation {{ w: 1.0 }} }} '
            f'relative_to: "{self._frame_id}"'
        )
        return self._call_ign_service(
            service=service,
            reqtype="ignition.msgs.EntityFactory",
            reptype="ignition.msgs.Boolean",
            request=request,
        )

    def _move_marker(self, x_pos: float, y_pos: float, z_pos: float) -> bool:
        service = f"/world/{self._world_name}/set_pose"
        request = (
            f'name: "{self._marker_name}" '
            f'position {{ x: {x_pos} y: {y_pos} z: {z_pos} }} '
            f'orientation {{ w: 1.0 }}'
        )
        return self._call_ign_service(
            service=service,
            reqtype="ignition.msgs.Pose",
            reptype="ignition.msgs.Boolean",
            request=request,
        )

    def _call_ign_service(
        self, service: str, reqtype: str, reptype: str, request: str
    ) -> bool:
        cmd = [
            "ign",
            "service",
            "-s",
            service,
            "--reqtype",
            reqtype,
            "--reptype",
            reptype,
            "--timeout",
            str(self._timeout_ms),
            "--req",
            request,
        ]
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=False,
            )
        except FileNotFoundError:
            self.get_logger().error("'ign' CLI not found on PATH.")
            return False

        stdout = result.stdout.strip()
        stderr = result.stderr.strip()
        if stdout:
            self.get_logger().info(f"ign stdout: {stdout}")
        if stderr:
            self.get_logger().warning(f"ign stderr: {stderr}")

        if result.returncode != 0:
            return False

        return "data: true" in stdout.lower() or stdout == ""

    def _build_marker_sdf(self) -> str:
        return (
            "<sdf version='1.7'>"
            f"<model name='{self._marker_name}'>"
            "<static>true</static>"
            "<link name='link'>"
            "<visual name='visual'>"
            "<geometry><sphere><radius>"
            f"{self._marker_radius}"
            "</radius></sphere></geometry>"
            "<material>"
            "<ambient>1 0 0 1</ambient>"
            "<diffuse>1 0 0 1</diffuse>"
            "<specular>0.2 0.2 0.2 1</specular>"
            "<emissive>0.6 0 0 1</emissive>"
            "</material>"
            "</visual>"
            "</link>"
            "</model>"
            "</sdf>"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartesianTargetMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
