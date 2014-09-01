#!/usr/bin/env python


import matplotlib.pyplot as plt
import numpy as np
import csv
import time

import argparse
parser = argparse.ArgumentParser(description='Show shapes in csv file');
parser.add_argument('input', action="store",
                help='a string containing the name of csv file to read from');
parser.add_argument('--no_clear', action="store_true",
                help="don't clear the display (useful for viewing shapes in proportion to each other");
args = parser.parse_args();

if args.no_clear:
    no_clear = True;
else:
    no_clear = False;
plt.ion();
with open(args.input, 'rb') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in csvreader:
        stroke = row[2:];
        
        #import pdb; pdb.set_trace()
        stroke = map(float, stroke)
        x_shape = stroke[0::2];
        y_shape = stroke[1::2];

        if(not no_clear):
            plt.clf()
        plt.plot(np.asarray(x_shape), np.asarray(y_shape), linewidth=10);
        plt.draw()
        time.sleep(1.0);

