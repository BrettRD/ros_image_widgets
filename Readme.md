# image_widgets
A simple ros2 package that publishes image topics with a dial or something.

Requires transparent PNG images to be provided in params.

Intended for use with gst_bridge to composite image topics into an instrument panel on a live stream


# run
```ros2 run image_widgets compass_widget_node --ros-args -p img_paths:="[src/image_widgets/images/compass_background.png, src/image_widgets/images/compass_needle.png]"```
```ros2 topic pub -1 /compass_pose geometry_msgs/msg/Twist '{angular:{z: 0}}'```
