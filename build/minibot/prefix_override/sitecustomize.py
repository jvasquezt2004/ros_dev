import sys
if sys.prefix == '/home/alonso/.local/share/mise/installs/python/3.12.11':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alonso/Development/School/ros_dev/install/minibot'
