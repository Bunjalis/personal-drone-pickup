import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mitchell/Documents/PhD/drone_cage_control/install/motion_capture_publisher'
