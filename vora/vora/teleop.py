# ROS imports
import rclpy
from rclpy.node import Node
from vora_interfaces.msg import VORACommand # type: ignore

from threading import Thread

class teleop(Node):
    """
    A node to control the neato-control node with typed commands
    """

    def __init__(self):
        """
        Initialize the node
        """

        super().__init__('teleop') # type: ignore

        # Start the publisher to communicate with the neato-control node
        self.vora_command = self.create_publisher(VORACommand, 'vora_command', 10)

        Thread(target=self.loop).start()

    def loop(self):
        """
        The main loop. Gets input from the user to send to the neato-control node. The "person"
        field of the command is currently always "teleop".
        """

        while True:
            # Get input from the user
            command = input("command: ")
            # person = input("person: ")
            person = "teleop"
            arg = float(input("arg: "))

            # Create a message, populate it, and publish it
            msg = VORACommand()
            msg.command = command
            msg.person = person
            msg.arg = arg
            self.vora_command.publish(msg)

if __name__ == '__main__':
    node = teleop()
    node.run() # type: ignore

def main(args=None):
    rclpy.init()
    n = teleop()
    rclpy.spin(n)
    rclpy.shutdown()