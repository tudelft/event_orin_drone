from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import torch
import torch.nn as nn


class Monodepth(Node):
    def __init__(self):
        super().__init__("monodepth")

        # subscribe to ~/image topic
        self.subscription = self.create_subscription(Image, "~/image", self.listener_callback, 10)
        self.bridge = CvBridge()

        # publish to ~/avg_inv_depth topic
        self.publisher = self.create_publisher(Float32MultiArray, "~/avg_inv_depth", 10)

        # init some random network
        # taking any shape -> producing 8 avg depths
        self.device, self.dtype = torch.device("cuda"), torch.float32
        self.network = nn.Sequential(
            nn.Conv2d(3, 8, 3, stride=2, padding=1),
            nn.ELU(),  # more expressive than ReLU
            nn.Conv2d(8, 16, 3, stride=2, padding=1),
            nn.ELU(),
            nn.Conv2d(16, 32, 3, stride=2, padding=1),  # 8x downsampled by now
            nn.ELU(),
            nn.AdaptiveAvgPool2d(1),  # allows taking any input shape
            nn.Flatten(),
            nn.Linear(32, 8),
            nn.Softplus(),  # to ensure smooth positive depth
        )
        self.network.to(self.device, dtype=self.dtype)
        self.network.eval()

    def listener_callback(self, msg):
        # ros image to opencv to torch
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img = (torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).float() / 255.0)
        img = img.to(self.device, dtype=self.dtype)

        # forward pass
        with torch.no_grad():
            avg_depth = self.network(img)
        avg_inv_depth = 1.0 / avg_depth

        # convert torch to ros msg and publish
        avg_inv_depth = avg_inv_depth.squeeze().detach().cpu().numpy()
        self.get_logger().info(f"avg_inv_depth: {[round(x, 2) for x in avg_inv_depth]}")
        msg = Float32MultiArray(data=avg_inv_depth)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    monodepth = Monodepth()
    rclpy.spin(monodepth)
    monodepth.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
