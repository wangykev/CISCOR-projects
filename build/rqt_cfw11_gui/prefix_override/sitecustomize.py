import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ciscor/CISCOR-projects/install/rqt_cfw11_gui'
