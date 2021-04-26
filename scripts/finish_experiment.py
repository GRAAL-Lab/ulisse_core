#!/usr/bin/env python
import os, sys

from os.path import expanduser
from shutil import copyfile

home = expanduser("~")

if len(sys.argv) < 2:
    print("Insert name of experiment")
    exit(0)

name = sys.argv[1]

directory = 'ulisse_ctrl/conf/'
main_dir = 'Experiments/'
new_dir = main_dir + name

if not os.path.isdir(main_dir):
    os.mkdir(main_dir)

if not os.path.isdir(new_dir):
    os.mkdir(new_dir)

for filename in os.listdir(directory):
	copyfile(directory+filename, new_dir+"/"+filename)

directory2 = home + "/log_ulisse/"
for filename in os.listdir(directory2):
	extension = os.path.splitext(filename)[1]
	if extension == ".csv":
		os.rename(directory2 + filename, new_dir+"/"+filename)