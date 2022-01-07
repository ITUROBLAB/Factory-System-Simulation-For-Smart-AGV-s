#!/usr/bin/env python3

import sys

from agv_similation_ui.sim_ui_plugin import AGVSimilationPlugin
from rqt_gui.main import Main


plugin = 'sim_ui_plugin'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))  