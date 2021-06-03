# image_widgets
use jinja2 to create complex strings from arbitrary message types

```ros2 run image_widgets jinjaturtle.py --ros-args -p message_module:="std_msgs.msg" -p message_class:="String" -p template_data:="hello {{msg.data}}"```  
```ros2 topic pub /raw_msg std_msgs/msg/String data:\ \'world\'\```

# "image_widgets" though?
image_widgets was originally intended for creating animated overlays over live camera topics in telepresence systems.  
SVG became the preferred parametric image format, jinja2 was used for loading parameters, and gstreamer was better for video compositing, leaving this package the job of feeding ROS2 messages to a templater, and nothing else.

Later enhancements might include a node that renders SVGs without a gstreamer dependency

# making pictures
```gst-launch-1.0 --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ rostextsrc topic=string_msg ! rsvgdec ! videoconvert ! rosimagesink```  
```ros2 launch image_widgets example.launch.py config_filename:=compass_inline_template```  
```ros2 topic pub /compass_pose geometry_msgs/msg/Twist '{angular:{z: 0}}'```


# overlay widgets onto a video stream.
```gst-launch-1.0 --gst-plugin-path=install/gst_bridge/lib/gst_bridge/  videotestsrc ! mix. rostextsrc topic=string_msg ! rsvgdec ! videoconvert ! compositor name=mix !  videoconvert ! autovideosink```
