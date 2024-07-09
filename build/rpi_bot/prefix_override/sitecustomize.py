import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/psu/RPi_Base/src/rpi_bot/install/rpi_bot'
