import rclpy
from rclpy.node import Node
from min.minmon import MINMonitor
from min import MINFrame
from min.PayloadBuilder import PayloadBuilder
from numpy import uint8
from sensor_msgs.msg import Imu, Range, Illuminance, Temperature, BatteryState
from std_msgs.msg import Bool
import logging

def convert_subdevice_mask_to_index(mask: uint8) -> uint8:
    for i in range(8):
        current_mask = 0x01 << i
        if (mask & current_mask) == current_mask:
            return i

class MobiCtlSensors(Node):

    def __init__(self):
        super().__init__('mobictl_publisher')
        # setup logger
        logging.getLogger("min").addHandler(RosHandler(self.get_logger()))

        self.min_mon: MINMonitor = MINMonitor(port="/dev/ttyACM0", loglevel=logging.INFO)
        self.get_logger().info("Started MobiCtl")

        # self.pub_imu = self.create_publisher(Imu, "mobictl/imu", 10)
        # self.last_imu = Imu()
        
        # self.pub_ultra_1 = self.create_publisher(Range, "mobictl/ultra_1", 10)
        # self.pub_ultra_2 = self.create_publisher(Range, "mobictl/ultra_2", 10)
        # self.pub_ultra_3 = self.create_publisher(Range, "mobictl/ultra_3", 10)
        # self.pub_ultra_4 = self.create_publisher(Range, "mobictl/ultra_4", 10)
        # self.pub_ultra_5 = self.create_publisher(Range, "mobictl/ultra_5", 10)
        # self.pub_ultra_6 = self.create_publisher(Range, "mobictl/ultra_6", 10)
        
        self.pub_brightness = self.create_publisher(Illuminance, "mobictl/brightness", 10)
        # self.pub_temperature = self.create_publisher(Temperature, "mobictl/temperature", 10)
        # self.pub_battery = self.create_publisher(BatteryState, "mobictl/battery", 10)
        # self.pub_user_btn = self.create_publisher(Bool, "mobictl/user_btn", 10)

        self.setup()
        self.loop()

        # On exit
        self.get_logger().info("Shutting down")
        
    def setup(self):
        # Reset all 
        self.min_mon.send_frame(61)
        # IMU
        # pb = PayloadBuilder()
        # pb.append_uint8(0b1010100)
        # pb.append_uint16(5000)
        # self.min_mon.send_frame(32, pb.get_payload())
        # # Ultraschallsensor
        # pb = PayloadBuilder()
        # pb.append_uint8(0x3F)
        # pb.append_uint16(5000)
        # self.min_mon.send_frame(33, pb.get_payload())
        # brightness    
        pb = PayloadBuilder()
        pb.append_uint16(200)
        self.min_mon.send_frame(35, pb.get_payload())
        # # temperature
        # pb = PayloadBuilder()
        # pb.append_uint16(2000)
        # self.min_mon.send_frame(36, pb.get_payload())
        # # battery
        # pb = PayloadBuilder()
        # pb.append_uint16(10000)
        # self.min_mon.send_frame(37, pb.get_payload())
        # # user button
        # self.min_mon.send_frame(38, bytes([1]))


    def loop(self):
        while rclpy.ok():
            while self.min_mon._recv_messages.qsize() > 0:
                receved_frame: MINFrame = self.min_mon.recv(block=False)
                self.decode_frame(receved_frame)

    def decode_frame(self, frame: MINFrame):
        pb = PayloadBuilder(frame.payload)

        # self.get_logger().info(f"decoding frame with id: {frame.min_id}")
        if frame.min_id == 0x01: # IMU
            sub_device = convert_subdevice_mask_to_index(pb.read_uint8())

            w = pb.read_double();
            x = pb.read_double();
            y = pb.read_double();
            z = pb.read_double();

            if sub_device == 2: # gyroscope
                self.last_imu.angular_velocity.x = x
                self.last_imu.angular_velocity.y = y
                self.last_imu.angular_velocity.z = z
            elif sub_device == 4: #linear_accel
                self.last_imu.linear_acceleration.x = x
                self.last_imu.linear_acceleration.y = y
                self.last_imu.linear_acceleration.z = z
            elif sub_device == 6: # quaternion
                self.last_imu.orientation.w = w
                self.last_imu.orientation.x = x
                self.last_imu.orientation.y = y
                self.last_imu.orientation.z = z
            
            self.last_imu.header.stamp = self.get_clock().now().to_msg()
            self.pub_imu.publish(self.last_imu)

        elif frame.min_id == 0x02: # US
            sub_device = convert_subdevice_mask_to_index(pb.read_uint8())

            msg = Range()
            msg.radiation_type = 0
            msg.range = pb.read_float().item()
            msg.min_range = 2.0
            msg.max_range = 400.0
            msg.header.stamp = self.get_clock().now().to_msg()

            if sub_device == 0:
                self.pub_ultra_1.publish(msg)
            if sub_device == 1:
                self.pub_ultra_2.publish(msg)
            if sub_device == 2:
                self.pub_ultra_3.publish(msg)
            if sub_device == 3:
                self.pub_ultra_4.publish(msg)
            if sub_device == 4:
                self.pub_ultra_5.publish(msg)
            if sub_device == 5:
                self.pub_ultra_6.publish(msg)
                
        elif frame.min_id == 0x04: # brightness
            msg = Illuminance()
            msg.illuminance = pb.read_float().item()
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_brightness.publish(msg)
        
        elif frame.min_id == 0x05: # temperature
            msg = Temperature()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.temperature = float(pb.read_int8().item())
            self.pub_temperature.publish(msg)

        elif frame.min_id == 0x06: # battery voltage
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.present = True
            msg.voltage = pb.read_float().item()
            self.pub_battery.publish(msg)
        
        elif frame.min_id == 0x07: # user button
            msg = Bool()
            msg.data = bool(pb.read_uint8().item())
            self.pub_user_btn.publish(msg)
        
        # clear pb
        pb.get_payload()
 
def main(args=None):
    rclpy.init(args=args)

    mobictl_publisher = MobiCtlSensors()

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)


    mobictl_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

class RosHandler(logging.Handler):
    def __init__(self, logger):
        super().__init__()
        self.logger = logger

    def emit(self, record):
        msg = self.format(record)
        if record.levelno >= logging.ERROR:
            self.logger.error(msg)
        elif record.levelno >= logging.WARNING:
            self.logger.warn(msg)
        else:
            self.logger.info(msg)