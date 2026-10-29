# Creates the plotter from config.py, like bg.py on the Pi:
#
#   >>> from bg import bg
#   >>> bg.box()
#   >>> bg.plot_file("/drawings/switzerland.lines")

from brachiograph import BrachioGraph
import config

bg = BrachioGraph(**config.PLOTTER)
