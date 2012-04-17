#-------------Imports----------------------------------------------------------#
from xml.dom.minidom import parse, parseString
from time import strftime
import logging
import glob
import os
import re
import sys
import shutil
import stat
from commands import getoutput as gop

#-------------Copy files to redist---------------------------------------------#
print "* Copying files to redist dir..."

#license
shutil.copy("../../GPL.txt", REDIST_DIR)
shutil.copy("../../LGPL.txt", REDIST_DIR)

#lib
if ostype == "Darwin":
    LIBS_TYPE = ".dylib"
else:
    LIBS_TYPE = ".so"

shutil.copy("Bin/" + PLATFORM + "-Release/libnimCodecs"+LIBS_TYPE, REDIST_DIR + "/Lib")
shutil.copy("Bin/" + PLATFORM + "-Release/libnimMockNodes"+LIBS_TYPE, REDIST_DIR + "/Lib")
shutil.copy("Bin/" + PLATFORM + "-Release/libnimRecorder"+LIBS_TYPE, REDIST_DIR + "/Lib")
shutil.copy("Bin/" + PLATFORM + "-Release/libOpenNI"+LIBS_TYPE, REDIST_DIR + "/Lib")
shutil.copy("Bin/" + PLATFORM + "-Release/libOpenNI.jni"+LIBS_TYPE, REDIST_DIR + "/Lib")

#bin
MonoDetected = 0
shutil.copy("Bin/" + PLATFORM + "-Release/niReg", REDIST_DIR + "/Bin")
shutil.copy("Bin/" + PLATFORM + "-Release/niLicense", REDIST_DIR + "/Bin")
if PLATFORM == 'x86' or PLATFORM == 'x64':
    if (os.path.exists("/usr/bin/gmcs")):
        shutil.copy("Bin/" + PLATFORM + "-Release/OpenNI.net.dll", REDIST_DIR + "/Bin")
        shutil.copy("Bin/" + PLATFORM + "-Release/OpenNI.net.dll", REDIST_DIR + "/Samples/Bin/" + PLATFORM + "-Debug")
        shutil.copy("Bin/" + PLATFORM + "-Release/OpenNI.net.dll", REDIST_DIR + "/Samples/Bin/" + PLATFORM + "-Release")
        MonoDetected = 1
        
# java wrapper
shutil.copy("Bin/" + PLATFORM + "-Release/org.OpenNI.jar", REDIST_DIR + "/Jar")
shutil.copy("Bin/" + PLATFORM + "-Release/org.OpenNI.jar", REDIST_DIR + "/Samples/Bin/" + PLATFORM + "-Debug")
shutil.copy("Bin/" + PLATFORM + "-Release/org.OpenNI.jar", REDIST_DIR + "/Samples/Bin/" + PLATFORM + "-Release")

#docs
shutil.copytree("../../Source/DoxyGen/html", REDIST_DIR + "/Documentation/html")

#include
for includeFile in os.listdir("../../Include"):
    if not os.path.isdir("../../Include/" + includeFile):
        shutil.copy("../../Include/" + includeFile, REDIST_DIR + "/Include")

shutil.copytree("../../Include/Linux-x86", REDIST_DIR + "/Include/Linux-x86")
shutil.copytree("../../Include/Linux-Arm", REDIST_DIR + "/Include/Linux-Arm")
shutil.copytree("../../Include/MacOSX", REDIST_DIR + "/Include/MacOSX")
shutil.copytree("Build/Common", REDIST_DIR + "/Samples/Build/Common")

# samples
samples_list = os.listdir("Build/Samples")
if '.svn' in samples_list:
    samples_list.remove('.svn')

if PLATFORM == 'CE4100':
    samples_list.remove('NiViewer')
    samples_list.remove('NiSimpleViewer')

if PLATFORM == 'Arm':
    samples_list.remove('NiUserTracker')
    samples_list.remove('NiViewer')
    samples_list.remove('NiSimpleViewer')
    samples_list.remove('NiHandTracker')
    samples_list.remove('NiUserSelection')

if (MonoDetected == 0):
    samples_list.remove("SimpleRead.net")
    samples_list.remove("SimpleViewer.net")
    samples_list.remove("UserTracker.net")

print "Samples:", samples_list

for sample in samples_list:
    shutil.copytree("../../Samples/" + sample, REDIST_DIR + "/Samples/" + sample)
    shutil.copy("Build/Samples/"+ sample + "/Makefile", REDIST_DIR + "/Samples/"+ sample)

#data
shutil.copy("../../Data/SamplesConfig.xml", REDIST_DIR + "/Samples/Config/SamplesConfig.xml")

#res
res_files = os.listdir("Build/Res")
if '.svn' in res_files:
    res_files.remove('.svn')
for res_file in res_files:
    shutil.copy("Build/Res/" + res_file, REDIST_DIR + "/Samples/Res")

# remove all .svn files
os.system("find " + REDIST_DIR + "/. | grep .svn | xargs rm -rf")

# remove all .svn files
os.system("find " + REDIST_DIR + "/Samples/. | grep .svn | xargs rm -rf")


#-------------Build Samples---------------------------------------------------#
print "* Building Samples in release configuration......"

# Build project solution
execute_check("make " + MAKE_ARGS + " -C " + REDIST_DIR + "/Samples/Build " + " > "+SCRIPT_DIR+"/Output/BuildSmpRelease.txt", "Build samples in release")

print "* Building Samples in debug configuration......"

# Build project solution
execute_check("make " + MAKE_ARGS + " CFG=Debug -C " + REDIST_DIR + "/Samples/Build > "+SCRIPT_DIR+"/Output/BuildSmpDebug.txt", "Build samples in debug")

# delete intermidiate files
for sample in samples_list:
   os.system("rm -rf " + REDIST_DIR + "/Samples/"+sample+"/" + PLATFORM + "/Debug")
   os.system("rm -rf " + REDIST_DIR + "/Samples/"+sample+"/" + PLATFORM + "/Release")


