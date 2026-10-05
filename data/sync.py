#!/usr/bin/env python3

import argparse
import os
import sys

from robotic.tools.data_installer import *


def main():
    parser = argparse.ArgumentParser(description="pull data from HAL; or push with -push")
    parser.add_argument("-pull", help="pull data", action="store_true", default=False)
    parser.add_argument("-push", help="push data", action="store_true", default=False)
    parser.add_argument("-f", help="actually execute sync instead of dry run", action="store_true", default=False)
    args = parser.parse_args()

    if not (args.push ^ args.pull):
        print("-- ERROR: you need to choose either -pull or -push")
        return

    D = DataInstaller("calibration/", "./", dry_run=(not args.f))
    if args.push:
        D.push()
    else:
        D.pull()

    if not args.f:
        print("\n== INFO: this was a dry run! Add -f to actually do the sync. ==\n")


if __name__ == "__main__":
    os.chdir(sys.path[0])  ##IMPORTANT - always relative to this pwd
    main()
