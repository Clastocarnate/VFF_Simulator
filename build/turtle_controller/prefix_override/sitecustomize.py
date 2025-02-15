import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/clasto/Documents/VFF_Simulator/install/turtle_controller'
