# ROS imports
import rclpy
from rclpy.node import Node
from vora_interfaces.msg import VORACommand # type: ignore

from threading import Thread

class test(Node):
    """
    TODO Write Docstrings
    """
    def __init__(self):
        super().__init__('neato_control') # type: ignore

        self.vora_command = self.create_publisher(VORACommand, 'vora_command', 10)

        Thread(target=self.loop).start()

    def loop(self):
        while True:
            command = input("command: ")
            # person = input("person: ")
            person = "test"
            arg = float(input("arg: "))

            msg = VORACommand()
            msg.command = command
            msg.person = person
            msg.arg = arg
            self.vora_command.publish(msg)

if __name__ == '__main__':
    node = test()
    node.run() # type: ignore

def main(args=None):
    rclpy.init()
    n = test()
    rclpy.spin(n)
    rclpy.shutdown()