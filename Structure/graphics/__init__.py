# Change matplotlib mode to no interactive. Otherwise, conficts will raise
# with the event of opencv.

import matplotlib
matplotlib.use('Agg')  # case-insensitive
