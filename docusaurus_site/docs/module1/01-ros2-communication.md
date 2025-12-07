---
sidebar_position: 1
sidebar_label: '1. ROS 2 Communication'
---

# Chapter 1: ROS 2 Communication

Welcome to the first module on ROS 2! This chapter covers the fundamental concepts that form the backbone of any robotic application in ROS 2: Nodes, Topics, and Services.

## Core Concepts

In ROS 2, a robot's software is built as a distributed system of small, independent programs called **Nodes**. These nodes communicate with each other using a set of well-defined patterns.

### Nodes

A **Node** is the smallest unit of execution in ROS 2. Think of it as a single-purpose program. One node might be responsible for reading from a laser scanner, another for controlling the wheel motors, and a third for planning a path. By breaking a complex system into many small nodes, the software becomes more modular, easier to debug, and reusable.

### Topics (Asynchronous, Many-to-Many)

**Topics** are the most common communication pattern. They provide a publish/subscribe mechanism for sending and receiving data.

-   A node that produces data **publishes** messages to a topic.
-   A node that needs that data **subscribes** to the same topic.

Topics are ideal for continuous data streams, like sensor readings, robot state, or commands. Many nodes can publish to the same topic, and many nodes can subscribe to it. The communication is asynchronous; the publisher doesn't know or care who is listening.

[A diagram illustrating topic communication will be placed here.]

### Services (Synchronous, One-to-One)

**Services** are used for synchronous, request/response communication. This is like a remote procedure call (RPC).

-   A node offers a **service server**, which can perform a specific task when called.
-   Another node acts as a **service client**, sending a request and waiting for a response.

Services are perfect for actions that have a clear start and end, like "compute a path," "grasp an object," or "save the current map." Unlike topics, a service call is a direct, one-to-one interaction, and the client waits until the server completes the request and sends back a result.

## Example: A Simple Publisher and Subscriber

Let's see this in action with a simple "Hello World" example using topics. We will create two nodes: one that publishes a greeting and another that subscribes to it.

### The Publisher (`simple_publisher.py`)

This node repeatedly publishes a `String` message to the `chatter` topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Code Breakdown**:
1.  We import the necessary libraries: `rclpy`, `Node`, and the `String` message type.
2.  Our `SimplePublisher` class inherits from `Node`.
3.  In the constructor, we call `super().__init__('simple_publisher')` to name our node.
4.  `self.create_publisher(String, 'chatter', 10)` creates a publisher that sends `String` messages on the `chatter` topic. The `10` is the queue size.
5.  `self.create_timer(0.5, self.timer_callback)` sets up a timer to call our `timer_callback` function every 0.5 seconds.
6.  The `timer_callback` creates a `String` message, fills it with data, and publishes it.

### The Subscriber (`simple_subscriber.py`)

This node subscribes to the `chatter` topic and prints the messages it receives.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.get_logger().info('Subscribing to topic "chatter"')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Code Breakdown**:
1.  In the constructor, we create a subscription with `self.create_subscription(...)`.
2.  We tell it to subscribe to `String` messages on the `chatter` topic.
3.  We provide `self.listener_callback` as the function to be executed whenever a new message arrives.
4.  The `listener_callback` function simply logs the received message's data to the console.

### How to Run the Example

Follow the instructions in the `quickstart.md` guide to build and run these nodes. You will open two terminals. In one, you'll run the subscriber, and in the other, you'll run the publisher. You will see the "Hello World" messages being sent and received in real-time.
