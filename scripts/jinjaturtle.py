#!/usr/bin/python3

# this node publishes formatted strings using jinja2.
# users can specify message type via parameters on load.
# this node was intended to create animated SVG dashboard widgets.

from importlib import import_module
from jinja2 import Template

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import ParameterNotDeclaredException


class Jinjaturtle(Node):
    template_data = 'templated: {{msg.data}}'
    template = Template(template_data)
    message_module_name = 'std_msgs.msg'
    message_class_name = 'String'
    message_class = getattr(import_module(message_module_name), message_class_name)

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

        self.declare_parameter(
            'message_module',
            self.message_module_name,
            ParameterDescriptor(description='The module containing the message class')
        )
        self.declare_parameter(
            'message_class',
            self.message_class_name,
            ParameterDescriptor(description='The message class')
        )
        # XXX test param type
        self.message_module_name = self.get_parameter('message_module').get_parameter_value().string_value
        self.message_class_name = self.get_parameter('message_class').get_parameter_value().string_value
        self.cb_params( [
            self.get_parameter('template_data'),
            self.get_parameter('template_file'),
        ])

        try:
            message_module = import_module(self.message_module_name)
            self.message_class = getattr(message_module, self.message_class_name)
        except AttributeError as err:
            self.get_logger().error(f"could not import {self.message_class_name} from {self.message_module_name}")
            raise(err)
        except ModuleNotFoundError as err:
            self.get_logger().error(f"could not find module {self.message_module_name}")
            raise(err)

        self.add_on_set_parameters_callback(self.cb_params)
        self.publisher_ = self.create_publisher(String, 'string_msg', 10)
        self.subscription = self.create_subscription(
            self.message_class,
            'raw_msg',
            self.sub_callback,
            10)
        self.get_logger().info(f"listenning for a {self.message_module_name} {self.message_class_name}")


    def sub_callback(self, incoming_msg):
        fmt_msg = String()
        fmt_msg.data = self.template.render(msg=incoming_msg)
        self.publisher_.publish(fmt_msg)


    def cb_params(self, data):
        for parameter in data:
            if parameter.name == "template_file":
                if parameter.type_ == Parameter.Type.STRING:
                    if(parameter.value != ''):
                        try:
                            with open(parameter.value) as j2_file_:
                                self.template_data = j2_file_.read()
                                self.template = Template(self.template_data)
                                self.get_logger().info("loaded template file")
                        except:
                            self.get_logger().error("load template file failed")
            if parameter.name == "template_data":
                if parameter.type_ == Parameter.Type.STRING:
                    try:
                        self.template_data = parameter.value
                        self.template = Template(self.template_data)
                        self.get_logger().info("loaded template data")
                    except:
                        self.get_logger().error("load template data failed")
            if parameter.name == "message_module":
                self.get_logger().error("jinjaturtle does not support changing subscription type after load")
            if parameter.name == "message_class":
                self.get_logger().error("jinjaturtle does not support changing subscription type after load")


        return SetParametersResult(successful=True)




def main(args=None):
    rclpy.init(args=args)
    jinjaturtle = Jinjaturtle()
    rclpy.spin(jinjaturtle)
    jinjaturtle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()