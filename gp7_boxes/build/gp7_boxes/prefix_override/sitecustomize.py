import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jeewantha/Desktop/GP7_Robot._Simulaion/gp7_boxes/install/gp7_boxes'
