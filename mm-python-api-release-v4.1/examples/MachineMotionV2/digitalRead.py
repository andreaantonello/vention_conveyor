import sys
sys.path.append("../..")
from MachineMotion import *

### This Python example reads digital inputs for MachineMotion v2. ###

mm = MachineMotion(machineMotionHwVersion=MACHINEMOTION_HW_VERSIONS.MMv2)

# Detect all connected digital IO Modules
detectedIOModules = mm.detectIOModules()

# Read and print all input values
if detectedIOModules is not None :
   for IO_Name, IO_NetworkID in detectedIOModules.items():
      readPins = [0, 1, 2, 3]
      for readPin in readPins:
          pinValue = mm.digitalRead(IO_NetworkID, readPin)
          print("Pin " + str(readPin) + " on " + IO_Name + " has value " + str(pinValue))
