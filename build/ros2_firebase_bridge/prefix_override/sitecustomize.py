import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspaces/mobility-ros2-firebase/install/ros2_firebase_bridge'
