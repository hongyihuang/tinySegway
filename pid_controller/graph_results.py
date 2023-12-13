#!/usr/bin/python3

import sys
import argparse
import re
import matplotlib.pyplot as plt

"""
Parse a csv file of Balboa bot data and plot a graph of angle vs anglerate
"""

#line_regex = re.compile("ang:(\d+) del:(\d+) spd:(\d+)")
line_regex = re.compile(" *(-?\d+) +(-?\d+)")
ANG_GRP = 1
DEL_GRP = 2
SPD_GRP = 3

def main():
    parser = argparse.ArgumentParser(description="Specify input file")
    parser.add_argument('data_file', type=str, help='input csv file')
    parser.add_argument('graph_file', type=str, help='output graph file')
    args = parser.parse_args()

    angles, angle_rates = fileToLists(args.data_file)

    graph(angles, angle_rates, args.graph_file)
    
    return

"""Returns a tuple of the file data in list format"""
def fileToLists(data_file):
    with open(data_file) as f:
        lines = f.readlines()

    angles = []
    angle_rates = []
    
    for line in lines:
        parsed = line_regex.match(line)
        #if parsed:
        angles += [int(parsed.group(ANG_GRP))]
        angle_rates += [int(parsed.group(DEL_GRP))]

    return (angles, angle_rates)

"""Plots data"""
def graph(angles, angle_rates, filename):
    plt.plot(angles, angle_rates)
    plt.xlabel("angle (deg)")
    plt.ylabel("angle rates (deg/s)")
    plt.savefig(filename)
    return

if __name__ == "__main__":
  main()