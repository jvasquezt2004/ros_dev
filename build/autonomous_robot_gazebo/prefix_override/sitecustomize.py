import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alonso/Development/School/ros_dev/install/autonomous_robot_gazebo'
