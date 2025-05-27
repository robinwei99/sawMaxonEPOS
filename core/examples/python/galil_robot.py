# Start Galil Controller

import os, sys, ctypes

# Set RTLD_GLOBAL flag for dynamic loading (on Linux)
try:
    flags = sys.getdlopenflags()
    sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)
except AttributeError as e:
    print('Skipping dlopen flags, ' + str(e))

import cisstCommonPython as cisstCommon

# Set up cisst logging system to print errors, warnings, and verbose (but not debug)
cisstCommon.cmnLogger.SetMask(cisstCommon.CMN_LOG_ALLOW_VERBOSE)
cisstCommon.cmnLogger.SetMaskFunction(cisstCommon.CMN_LOG_ALLOW_VERBOSE)
cisstCommon.cmnLogger.SetMaskDefaultLog(cisstCommon.CMN_LOG_ALLOW_VERBOSE)
cisstCommon.cmnLogger.AddChannelToStdOut(cisstCommon.CMN_LOG_ALLOW_ERRORS_AND_WARNINGS)

def log():
    os.system('tail cisstLog.txt')

import cisstMultiTaskPython as cisstMultiTask
import cisstParameterTypesPython as cisstParameterTypes
import numpy

LCM = cisstMultiTask.mtsManagerLocal.GetInstance()
LCM.CreateAll()
LCM.StartAll()

arg = cisstMultiTask.mtsTaskContinuousConstructorArg('GalilServer', 256, True)
GalilServer = cisstMultiTask.mtsLoadAndCreateServer('sawGalilController',
                                                    'mtsGalilController',
                                                    'GalilServer', arg)

if GalilServer:
    print('Configuring Galil server.')
    # Python2 uses raw_input and Python3 uses input
    try:
        configFile = raw_input('Enter config filename (JSON): ')
    except NameError:
        configFile = input('Enter config filename (JSON): ')
    GalilServer.Configure(configFile)
    # robot is the required interface
    robot = cisstMultiTask.mtsCreateClientInterface('GalilClient', 'GalilServer', 'control')

LCM.CreateAllAndWait(2.0)
LCM.StartAllAndWait(2.0)

print('System ready. Type dir(robot) to see available commands.')
