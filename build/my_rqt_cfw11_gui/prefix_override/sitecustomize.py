import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ciscor/CISCOR-projects/install/my_rqt_cfw11_gui'
