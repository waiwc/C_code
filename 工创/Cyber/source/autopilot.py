import os, sys, getopt
import sys
import shutil

if sys.version_info[0] < 3:
  import urllib as ur
else:
  import urllib.request as ur

import subprocess
import traceback

# defines constants
GLOBAL_CONFIG_PATH='/apollo/modules/common/data/global_flagfile.txt'

def conf_apollo_map(simone_od):
    if not os.path.isfile(simone_od):
        print("%s not exist" %(simone_od))
        return
    # copy simone xodr to apollo map folder Ex: /apollo/modules/map/data/Junction3
    od = simone_od.split('/')
    apollo_od = "/apollo/modules/map/data/" + od[2]
    if not os.path.exists(apollo_od):
        os.makedirs(apollo_od)
    shutil.copy(simone_od, apollo_od)
    # config map_dir in global_flagfile.txt
    replace_prefix = "--map_dir="
    global_flagfile = "/apollo/modules/common/data/global_flagfile.txt"

    with open(global_flagfile, "r", encoding='utf-8') as fr:
        lines = fr.readlines()
    with open(global_flagfile, "w", encoding='utf-8') as fw:
        for line in lines:
            if replace_prefix in line:
                line = line.replace(line, replace_prefix + apollo_od)
            fw.write(line)
    fr.close()
    fw.close()

def killApollo():
    print("Apollo Exiting ......")
    subprocess.call(["bash", "./kill.sh"])

def runApollo(simone_od):
    conf_apollo_map(simone_od)
    print("Apollo Starting ......")
    subprocess.call(["bash", "./run.sh"])

def main():
    try:
        opts, args = getopt.getopt(sys.argv[1:], "ko:",
            ["od="])
        for op, value in opts:
            if op in ("-o", "--od"):
                runApollo(value)
            elif op in ("-k"):
                killApollo()
    except Exception as e:
        print(e)
        print(sys.stderr)
        traceback.print_exc()
        sys.exit()

if __name__ == '__main__':
    main()
