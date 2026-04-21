import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import base64
import tempfile
import socket

class TPMGatekeeper(Node):
    def __init__(self):
        super().__init__('tpm_gatekeeper')

        self.pub_status = self.create_publisher(String, '/auth/status', 10)
        self.pub_response = self.create_publisher(String, '/auth/response', 10)

        self.sub_challenge = self.create_subscription(String, '/auth/challenge', self.challenge_callback, 10)
        self.sub_unlock = self.create_subscription(String, '/auth/unlock', self.unlock_callback, 10)

        # Include hostname in heartbeat so the controller knows which robot
        # is connecting and can load the correct public key.
        self.robot_id = os.getenv('USER', socket.gethostname())

        self.auth_timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f"TPM Gatekeeper active ({self.robot_id}). Broadcasting 'READY' heartbeat...")

    def publish_status(self):
        msg = String()
        # Format: "ROBOT_READY:<hostname>" so controller can look up the right key.
        msg.data = f"ROBOT_READY:{self.robot_id}"
        self.pub_status.publish(msg)

    def challenge_callback(self, msg):
        # Cancel the heartbeat timer while processing the challenge
        # so we don't keep broadcasting ROBOT_READY during signing.
        if self.auth_timer:
            self.auth_timer.cancel()
            self.auth_timer = None

        challenge_string = msg.data

        # Use a temp directory instead of the current working directory.
        # ROS 2 nodes have no guaranteed cwd, so writing relative paths can fail or
        # land in unexpected places. tempfile.mkdtemp() gives us a safe, known location.
        tmp_dir = tempfile.mkdtemp(prefix="tpm_auth_")
        challenge_path = os.path.join(tmp_dir, "challenge.bin")
        hash_path = os.path.join(tmp_dir, "hash.bin")
        sig_path = os.path.join(tmp_dir, "sig.bin")

        try:
            # 1. Create a raw binary file of the challenge
            with open(challenge_path, "wb") as f:
                f.write(challenge_string.encode('utf-8'))

            # 2. Manually hash it first so we know EXACTLY what is being signed.
            # timeout=10 so a hung openssl call doesn't freeze the node.
            subprocess.run(
                ["openssl", "dgst", "-sha256", "-binary", "-out", hash_path, challenge_path],
                check=True,
                timeout=10
            )

            # 3. Sign the HASH, not the file (more reliable across platforms).
            cmd = [
                "tpm2_sign",
                "-c", "0x81010002",
                "-g", "sha256",
                "-d", hash_path,  # -d tells TPM this is already a digest
                "-f", "plain",    # plain format is easiest for Python to read
                "-o", sig_path
            ]
            # timeout=15 for the TPM call — hardware ops can be slow
            # but should never hang forever.
            subprocess.run(cmd, check=True, timeout=15)

            # 4. Encode and send
            with open(sig_path, "rb") as f:
                encoded_sig = base64.b64encode(f.read()).decode('utf-8')

            response_msg = String()
            response_msg.data = encoded_sig
            self.pub_response.publish(response_msg)
            self.get_logger().info("TPM signature published successfully.")

        except subprocess.TimeoutExpired as e:
            self.get_logger().error(f"TPM Signing timed out: {e}")
            self._publish_error("AUTH_ERROR: Signing timed out")

        except Exception as e:
            self.get_logger().error(f"TPM Signing failed: {e}")
            self._publish_error(f"AUTH_ERROR: {str(e)}")

        finally:
            # Always clean up temp files, even if signing failed.
            # Leaving signed challenge data on disk is a security risk.
            for path in [challenge_path, hash_path, sig_path]:
                if os.path.exists(path):
                    os.remove(path)
            if os.path.isdir(tmp_dir):
                os.rmdir(tmp_dir)

    def _publish_error(self, error_msg: str):
        """Helper to publish an error response on /auth/response."""
        err = String()
        err.data = error_msg
        self.pub_response.publish(err)

    def unlock_callback(self, msg):
        self.get_logger().info(f"Checking for unlock signal... Received: {msg.data}")

        if msg.data == "AUTH_SUCCESS":
            self.get_logger().info("Hardware Authenticated! Booting TurtleBot3...")

            # Stop heartbeat timer if it is somehow still running
            if self.auth_timer:
                self.auth_timer.cancel()
                self.auth_timer = None

            # Launch the robot using the custom namespaced launch file.
            # os.path.expanduser resolves '~' so this works regardless of the username.
            launch_file = os.path.expanduser(
                '~/turtlebot3_ws/src/turtlebot3/turtlebot3_bringup/launch/robot.launch.py'
            )
            subprocess.Popen(['ros2', 'launch', launch_file])

            # Destroy subscriptions so we don't launch it twice
            self.destroy_subscription(self.sub_challenge)
            self.destroy_subscription(self.sub_unlock)

def main(args=None):
    rclpy.init(args=args)
    gatekeeper = TPMGatekeeper()
    rclpy.spin(gatekeeper)
    gatekeeper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
