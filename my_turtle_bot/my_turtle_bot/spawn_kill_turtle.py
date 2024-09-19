#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from std_srvs.srv import Empty
from rclpy.executors import SingleThreadedExecutor
from functools import partial
from time import sleep
    
    
class SpawnNode(Node):
    def __init__(self):
        super().__init__("spawn_turtle") 
        self.spawn_turtle(5.54, 5.54, 0.0, "TurtleBot3")

    def spawn_turtle(self, x, y, theta, name):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for Turtlesim Node to Start ...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(self.callback_spawn_turtle)

    def callback_spawn_turtle(self, future):
        try:
            response = future.result()
            self.get_logger().info("Spawned : " + str(response.name))
        except Exception as e:
            self.get_logger().error("Spawning Request Failed %r " % (e,))


class KillNode(Node):
    def __init__(self):
        super().__init__("kill_turtle") 
        self.kill_turtle("turtle1")

    def kill_turtle(self, name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for Turtlesim Node to Start ...")

        request = Kill.Request()
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_kill_turtle, name=name))

    def callback_kill_turtle(self, future, name):
        try:
            response = future.result()
            #Gives Empty Response /kill
            self.get_logger().info("Killed : " + str(name))
        except Exception as e:
            self.get_logger().error("Killing Request Failed %r " % (e,))

class ClearNode(Node):
    def __init__(self):
        super().__init__("clear_path") 

    def clear_path(self):
        client = self.create_client(Empty, "clear")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for Turtlesim Node to Start ...")

        request = Empty.Request()

        future = client.call_async(request)
        future.add_done_callback(self.callback_clear_path)

    def callback_clear_path(self, future):
        try:
            response = future.result()
            #Gives Empty Response /kill
            self.get_logger().info("Cleared Path")
        except Exception as e:
            self.get_logger().error("Killing Request Failed %r " % (e,))

    
def main(args=None):
    rclpy.init(args=args)
    spawnnode = SpawnNode()
    killnode = KillNode()
    clearnode = ClearNode()
    executor = SingleThreadedExecutor()
    executor.add_node(spawnnode)
    executor.add_node(killnode)
    executor.add_node(clearnode)
    executor.spin()
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
