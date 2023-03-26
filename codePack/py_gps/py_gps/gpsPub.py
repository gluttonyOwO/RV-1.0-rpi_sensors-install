import time

import rclpy
from rclpy.node import Node
from vehicle_interfaces.msg import GPS

from gps3.agps3threaded import AGPS3mechanism

class Params(Node):
    def __init__(self, nodeName : str):
        super().__init__(nodeName)
        self.topic_GPS_nodeName = 'gps_publisher_node'
        self.topic_GPS_topicName = 'topic_GPS'
        self.topic_GPS_pubInterval = 0.5
        self.mainNodeName = 'gps_node'

        self.declare_parameter('topic_GPS_nodeName', self.topic_GPS_nodeName)
        self.declare_parameter('topic_GPS_topicName', self.topic_GPS_topicName)
        self.declare_parameter('topic_GPS_pubInterval', self.topic_GPS_pubInterval)
        self.declare_parameter('mainNodeName', self.mainNodeName)
        self._getParam()
    
    def _getParam(self):
        self.topic_GPS_nodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_GPS_nodeName').get_parameter_value())
        self.topic_GPS_topicName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_GPS_topicName').get_parameter_value())
        self.topic_GPS_pubInterval = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_GPS_pubInterval').get_parameter_value())
        self.mainNodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('mainNodeName').get_parameter_value())

class GPSPublisher(Node):

    def __init__(self, nodeName, topicName, interval_s):
        super().__init__(nodeName)
        self.nodeName_ = nodeName
        self.publisher_ = self.create_publisher(GPS, topicName, 10)
        self.frame_id_ = 0
        timer_period = interval_s  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.gpsThread = AGPS3mechanism()
        self.gpsThread.stream_data()
        self.gpsThread.run_thread()

    def timer_callback(self):
        msg = GPS()
        msg.header.priority = msg.header.PRIORITY_SENSOR
        msg.header.device_type = msg.header.DEVTYPE_GPS
        msg.header.device_id = self.nodeName_
        msg.header.frame_id = self.frame_id_
        self.frame_id_ += 1
        msg.header.stamp_type = msg.header.STAMPTYPE_NO_SYNC
        msg.header.stamp = self.get_clock().now().to_msg()

        
        if (self.gpsThread.data_stream.lat != 'n/a' and self.gpsThread.data_stream.lon != 'n/a'):
            msg.gps_status = GPS.GPS_STABLE
            # gpsTime = self.gpsThread.data_stream.time
            msg.latitude = self.gpsThread.data_stream.lat
            msg.longitude = self.gpsThread.data_stream.lon
            # speed = self.gpsThread.data_stream.speed
            # track = self.gpsThread.data_stream.track

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f, %f"' %(msg.latitude, msg.longitude))


def main(args=None):
    rclpy.init(args=args)
    params = Params('gps_params_node')
    gps_publisher = GPSPublisher(params.mainNodeName, params.topic_GPS_topicName, params.topic_GPS_pubInterval)
    rclpy.spin(gps_publisher)

    gps_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()