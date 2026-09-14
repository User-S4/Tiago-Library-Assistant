import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/opt/erc_ws/install/pal_pro_gripper_wrapper'
