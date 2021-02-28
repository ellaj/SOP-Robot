import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32



class JawDataPublisher(Node):

    def __init__(self):
        super().__init__('jaw_data_publisher')
        self.publisher_ = self.create_publisher(Float32, 'move_jaw', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.open = True
        self.pos = 0.0

    def timer_callback(self):
        msg = Float32()
        msg.data = self.pos
        if self.open:
           self.pos += 0.05
           if self.pos > 0.2:
               self.open = False 
        else:
            self.pos -= 0.05
            if self.pos < 0.05:
                self.open = True

        self.publisher_.publish(msg)

        self.get_logger().info('Publishing: %f' % msg.data)
        

def main(args=None):
    rclpy.init(args=args)

    jaw_data_publisher = JawDataPublisher()

    rclpy.spin(jaw_data_publisher)

    jaw_data_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
