import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yoga-slim-pro-2/robotarm_ws/install/arduinobot_python'
