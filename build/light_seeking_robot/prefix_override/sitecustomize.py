import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sura-itana/gazebo_practice/version2.../light_seeking_robot_ws/install/light_seeking_robot'
