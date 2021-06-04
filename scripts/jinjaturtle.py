#!/usr/bin/python3

# this node publishes formatted strings using jinja2.
# users can specify message type via parameters on load.
# this node was intended to create animated SVG dashboard widgets.

from importlib import import_module
from jinja2 import Template

import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.exceptions import ParameterAlreadyDeclaredException

from ament_index_python.packages import get_package_share_directory, get_package_prefix



class Jinjaturtle(Node):
    template_data = 'templated: {{msg.data}}'
    template_params = {}
    template = Template(template_data)
    template_file_path = os.path.join(get_package_share_directory('image_widgets'), 'images')
    message_module_name = 'std_msgs.msg'
    message_class_name = 'String'
    message_class = getattr(import_module(message_module_name), message_class_name)
    last_message = None

    def __init__(self):
        super().__init__('jinjaturtle',
        automatically_declare_parameters_from_overrides=True)
        #allow_undeclared_parameters=True)

        #self.get_logger().info(str(self.get_parameters_by_prefix('template_params')))
        #self.get_logger().info(  str(os.listdir(self.template_file_path)) )

        param_list = [
            (   'message_module', self.message_module_name,
                ParameterDescriptor(description='The module containing the message class')),
            (   'message_class', self.message_class_name,
                ParameterDescriptor(description='The message class')),
            (   'template_file', '',
                ParameterDescriptor(description='a path to a jinja2 template file')),
            (  'template_data', None,
                ParameterDescriptor(description='a jinja2 template')),
            (   'template_params', None,
                ParameterDescriptor(description='Namespace for additional parameters to send to the template')),
        ]
        for (param_name, param_value, param_descr)  in param_list:
            try:
                self.declare_parameter(param_name, param_value, param_descr)
            except ParameterAlreadyDeclaredException:
                self.set_descriptor(param_name, param_descr)

        if self.get_parameter('template_params').value != None:
            self.get_logger().warn('template_params is used as a namespace, ignoring this parameter')
            # XXX should we attempt to jsonloads this?


        for param_name, param in self.get_parameters_by_prefix('template_params').items():
            self.get_logger().info(f'found parameter {param_name} = {param.value}')
            self.template_params[param_name] = param.value


        # set the message type 
        # XXX test param type is actually a string 
        # try:
        self.message_module_name = self.get_parameter('message_module').get_parameter_value().string_value
        self.message_class_name = self.get_parameter('message_class').get_parameter_value().string_value
        # except Exception as err:
        # self.get_logger().error(f"message types must be strings {str(self.message_module_name)} {str(self.message_class_name)}")
        # raise err

        try:
            message_module = import_module(self.message_module_name)
            self.message_class = getattr(message_module, self.message_class_name)
        except AttributeError as err:
            self.get_logger().error(f"could not import {self.message_class_name} from {self.message_module_name}")
            raise(err)
        except ModuleNotFoundError as err:
            self.get_logger().error(f"could not find module {self.message_module_name}")
            raise(err)


        # we automatically declare override to fill the template_params namespace, so 
        #   we can't register the callback before declaration.

        # register the callback
        self.add_on_set_parameters_callback(self.cb_params)

        # run the callback to load the template
        self.cb_params( [
            self.get_parameter('template_data'),
            self.get_parameter('template_file'),
            self.get_parameter('template_params'),
        ])

        #register the pub/sub
        self.publisher_ = self.create_publisher(String, 'string_msg', 10)
        self.subscription = self.create_subscription(
            self.message_class,
            'raw_msg',
            self.sub_callback,
            10)
        self.get_logger().info(f"listenning for a {self.message_module_name} {self.message_class_name}")


    def sub_callback(self, incoming_msg):
        self.last_message = incoming_msg
        self.pub_message(incoming_msg)


    def pub_message(self, incoming_msg):
        fmt_msg = String()
        fmt_msg.data = self.template.render(msg=incoming_msg, params=self.template_params)
        self.publisher_.publish(fmt_msg)




    def cb_params(self, data):
        republish = False
        for parameter in data:
            if parameter.name == "template_file":
                if parameter.type_ == Parameter.Type.STRING:
                    if(parameter.value != ''):
                        try:
                            filename = os.path.join(self.template_file_path, parameter.value)
                            self.get_logger().info(f"loading filename {filename}")

                            with open(filename) as j2_file_:
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

            if parameter.name == "template_params":
                if parameter.value != None:
                    self.get_logger().warn("ignoring param 'template_params', try 'template_params.foo' ")
                    # XXX jsonloads?
            elif parameter.name.startswith("template_params"):
                param_name = parameter.name[len("template_params."):]
                if param_name in self.template_params:
                    if self.template_params[param_name] != parameter.value:
                        republish = True
                        self.template_params[param_name] = parameter.value

            if parameter.name == "message_module":
                self.get_logger().error("jinjaturtle does not support changing subscription type after load")

            if parameter.name == "message_class":
                self.get_logger().error("jinjaturtle does not support changing subscription type after load")

        if republish:
            if self.last_message != None:
                self.pub_message(self.last_message)


        return SetParametersResult(successful=True)




def main(args=None):
    rclpy.init(args=args)
    jinjaturtle = Jinjaturtle()
    rclpy.spin(jinjaturtle)
    jinjaturtle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()