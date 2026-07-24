import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mc509/Workspace/VLA/quest/questVR_ws_ros2/install'
