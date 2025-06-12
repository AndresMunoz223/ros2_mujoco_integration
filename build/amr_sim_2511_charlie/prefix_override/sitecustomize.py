import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/eia/ros2_mujoco_integration/install/amr_sim_2511_charlie'
