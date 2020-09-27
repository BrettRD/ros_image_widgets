# image_widgets
A simple visualisation tool that takes messages in and publishes image topics with a dial or something


# run
ros2 run image_widgets compass_widget_node --ros-args -p img_paths:="[src/image_widgets/images/compass_background.png, src/image_widgets/images/compass_needle.png]"
ros2 topic pub -1 /compass_pose geometry_msgs/msg/Twist '{angular:{z: 0}}'
