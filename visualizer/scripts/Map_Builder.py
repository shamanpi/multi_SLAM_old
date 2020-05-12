#! /usr/bin/env python
# Ashwad Pandit

import rospy
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import sys
from matplotlib.patches import Ellipse

from feature_extraction.msg import corners_and_lines

def my_range(start, end, step):
    while start <= end:
        yield start
        start += step
