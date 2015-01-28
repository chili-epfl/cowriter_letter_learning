#! /usr/bin/env python

from ast import literal_eval

import sys
import re


#action = re.compile('.*INFO - (?P<letter>\w):.*(?P<type>demonstration|generated|learning).*arams: (?P<params>\[[\d., -]*\]). Path: (?P<path>\[[\d., -]*\])')
action = re.compile('.*INFO - (?P<path>\[.*\])')

SCALE = 2000

if __name__ == "__main__":

    print("""<?xml version="1.0" standalone="no"?>
<svg width="210mm" height="297mm" version="1.1" xmlns="http://www.w3.org/2000/svg">
""")

    with open(sys.argv[1], 'r') as log:

        for line in log.readlines():

            found = action.search(line)
            if found:
                paths = literal_eval(found.group('path'))
                print("\t<g>")
                for path in paths:
                    sys.stdout.write("\t\t<path d=\"M%s %s " % (path[0][0] * SCALE, path[0][1] * SCALE))
                    for x,y in path:
                        x *= SCALE
                        y *= SCALE
                        sys.stdout.write("L%s %s " % (x, y))
                    sys.stdout.write("\" style=\"fill:none;stroke:#000000;stroke-width:2.0\"/>")
                print("\t</g>")


    print("</svg>")
