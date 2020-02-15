# test:
from FRC_utils.ballistics import calculate_vector, BallisticsGraph
from math import *

a, b = calculate_vector((0, 0, 0), (4, 0, 2.5))
print(degrees(a), b)
