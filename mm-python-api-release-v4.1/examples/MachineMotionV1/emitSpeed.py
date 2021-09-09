import sys
sys.path.append("../..")
from MachineMotion import *

### This Python example configures system speed for MachineMotion v1. ###

mm = MachineMotion()

speed = 50     # The max speed [mm/s] that all subsequent moves will move at
mm.emitSpeed(speed)
print("Global speed set to " + str(speed) + "mm/s.")
