---
sidebar_position: 2
sidebar_label: '2. Python Control with rclpy'
---

# Chapter 2: Python Control with rclpy

In the previous chapter, we learned about the core communication patterns in ROS 2. Now, let's dive deeper into writing our own nodes using `rclpy`, the official Python client library for ROS 2. This chapter will focus on creating a service server and client.

## A Basic `rclpy` Node

As we saw in the publisher/subscriber example, a basic Python node has a few key ingredients:
1.  **Imports**: `rclpy` and the necessary message or service types.
2.  **Node Class**: A class that inherits from `rclpy.node.Node`.
3.  **Constructor**: Initializes the node, gives it a name, and creates any publishers, subscribers, services, or clients.
4.  **Callbacks**: Functions that are executed when a message is received or a service is called.
5.  **Main function**: Initializes `rclpy`, creates an instance of your node, and "spins" it to keep it alive and processing events.

## Example: A Simple Service Server and Client

Services are used for request/response interactions. Let's create a service that adds two integers together.

### The Service Server (`simple_service_server.py`)

This node will provide a service named `add_two_ints` that, when called, will add two integers and return the sum.

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service "add_two_ints" is ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}\nSum: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Code Breakdown**:
1.  We import `AddTwoInts` from `example_interfaces.srv`. This is a standard service type provided by ROS 2 for examples. It defines a request with two integers (`a`, `b`) and a response with one integer (`sum`).
2.  In the constructor, `self.create_service()` creates the service server.
3.  We pass the service type (`AddTwoInts`), the service name (`add_two_ints`), and the callback function (`add_two_ints_callback`) to be executed upon receiving a request.
4.  The `add_two_ints_callback` function receives the `request` object (containing the input data) and the `response` object (which it must fill out).
5.  It calculates the sum, places it in `response.sum`, logs the transaction, and returns the modified `response` object.

### The Service Client (`simple_service_client.py`)

This node will act as a client, calling the `add_two_ints` service with numbers provided as command-line arguments.

```python
import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__('simple_service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print("Usage: ros2 run ros2_basics_examples simple_service_client <int1> <int2>")
        return

    simple_service_client = SimpleServiceClient()
    simple_service_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(simple_service_client)
        if simple_service_client.future.done():
            try:
                response = simple_service_client.future.result()
            except Exception as e:
                simple_service_client.get_logger().info(
                    f'Service call failed {e}')
            else:
                simple_service_client.get_logger().info(
                    f'Result of add_two_ints: for {simple_service_client.req.a} + {simple_service_client.req.b} = {response.sum}')
            break

    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Code Breakdown**:
1.  `self.create_client()` creates a client for the `add_two_ints` service.
2.  The `while` loop waits until the service is available before proceeding.
3.  `send_request` populates the request object with integers from the command-line arguments (`sys.argv`) and makes an asynchronous call with `self.client.call_async()`.
4.  This returns a `future` object that will hold the response when it arrives.
5.  The `main` function spins the node (`rclpy.spin_once`) until the future is `done()`, then retrieves the result and logs it.

### How to Run the Example

1.  Build your package with `colcon build`.
2.  Open a new terminal and source the workspace: `source install/setup.bash`.
3.  Run the service server: `ros2 run ros2_basics_examples simple_service_server`.
4.  Open a second terminal and source the workspace again.
5.  Run the client with two numbers: `ros2 run ros2_basics_examples simple_service_client 5 10`.
6.  The client will print the result: `Result of add_two_ints: for 5 + 10 = 15`. The server terminal will show the log from its callback.

```
