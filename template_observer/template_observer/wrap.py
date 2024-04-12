 #!/usr/bin/env python3

import numpy as np

def wrap(yaw):

    # Enter your code here
    yaw = np.arctan2(np.sin(yaw), np.cos(yaw))

    return yaw