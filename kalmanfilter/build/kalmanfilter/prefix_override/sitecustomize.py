import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ryosuke/ros2_ws/src/kalmanfilter/install/kalmanfilter'
