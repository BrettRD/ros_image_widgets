#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import ParameterNotDeclaredException

from jinja2 import Template
import yaml

class Jinjaturtle(Node):
    template_data = 'templated: {{msg.data}}'
    template = Template(template_data)
    def __init__(self):
        super().__init__('jinjaturtle')
        self.declare_parameter(
            'template_file',
            '',
            ParameterDescriptor(description='a path to a jinja2 template file')
        )
        self.declare_parameter(
            'template_data',
            self.template_data,
            ParameterDescriptor(description='a jinja2 template')
        )

        self.add_on_set_parameters_callback(self.cb_params)
        self.publisher_ = self.create_publisher(String, 'image_svg', 10)
        self.subscription = self.create_subscription(
            String,
            'cmd_vel',
            self.sub_callback,
            10)
        self.subscription  # prevent unused variable warning

    def sub_callback(self, geom_msg):
        fmt_msg = String()
        fmt_msg.data = self.template.render(msg=geom_msg)
        self.publisher_.publish(fmt_msg)
        #self.get_logger().info('Publishing: "%s"' % fmt_msg.data)


    def cb_params(self, data):
        for parameter in data:
            if parameter.name == "template_file":
                if parameter.type_ == Parameter.Type.STRING:
                    try:
                        with open(parameter.value) as j2_file_:
                            self.template_data = j2_file_.read()
                            self.template = Template(self.template_data)
                            self.get_logger().info("template changed ok")
                    except:
                        self.get_logger().error("template change failed")
            if parameter.name == "template_data":
                if parameter.type_ == Parameter.Type.STRING:
                    try:
                        self.template_data = parameter.value
                        self.template = Template(self.template_data)
                        self.get_logger().info("template changed ok")
                    except:
                        self.get_logger().error("template change failed")

        return SetParametersResult(successful=True)




def main(args=None):
    rclpy.init(args=args)
    jinjaturtle = Jinjaturtle()
    rclpy.spin(jinjaturtle)
    jinjaturtle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()