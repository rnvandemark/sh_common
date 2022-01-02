from rclpy.node import Node
from std_msgs.msg import Header

import sh_common_constants

PARAM_HEARTBEAT_PERIOD_MS = sh_common_constants.params.HEARTBEAT_PERIOD_MS

## A node which periodically publishes a simple heartbeat to inform others that
#  it is alive and well.
class HeartbeatNode(Node):

    ## The constructor. Implements the node heartbeat and creates the action server.
    #  @param self The object pointer.
    def __init__(self, node_name, ns=""):
        super(HeartbeatNode, self).__init__(node_name, namespace=ns)

        #
        # Replicate HeartbeatNode functionality
        #

        self.declare_parameters(
            namespace="",
            parameters=[
                (PARAM_HEARTBEAT_PERIOD_MS, -1),
            ]
        )

        self.heartbeat_pub = self.create_publisher(
            Header,
            sh_common_constants.topics.HEARTBEAT_PREFIX + "/" + self.get_name(),
            10
        )

        self.heartbeat_msg = Header()
        self.heartbeat_msg.frame_id = self.get_name()
        self.heartbeat_timer = self.create_timer(
            self.get_parameter(PARAM_HEARTBEAT_PERIOD_MS).value / 1000,
            self.heartbeat_callback
        )

    ## Simple callback for the heartbeat message.
    #  @param self The object pointer.
    def heartbeat_callback(self):
        self.heartbeat_msg.stamp = self.get_clock().now().to_msg()
        self.heartbeat_pub.publish(self.heartbeat_msg)
