# python file for performing multiple analyses on latency data.
# the CSV file should be of the form:
# filename, controller name, battery life
#
# This script will output a horizontal box plot of the latency data for each
# controller, where the x axis is the latency in ms and the y axis is the
# battery life in hours. there will be a legend matching the controller name to
# the color of the box plot.

import sys
import numpy as np
import matplotlib.pyplot as plt

import argparse

from analysis import load_file

def main():
    parser = argparse.ArgumentParser(description='Analyze latency data.')
    parser.add_argument('csv', type=str, help='The csv file containing a csv of the form: filename, controller name, battery life')
    parser.add_argument('--title', type=str, help='The title of the plot.', default='Latency Box Plot')
    args = parser.parse_args()

    # if no filename is provided, print an error and exit
    if not args.csv:
        print('Error: no csv file provided.')
        sys.exit(1)

    # load the file, parse all the rows for the files to analyze
    files = {}
    try:
        with open(args.csv, 'r') as f:
            lines = f.readlines()
            # skip the header line
            lines = lines[1:]
            for line in lines:
                filename, controller, battery = line.split(',')
                files[filename] = (controller, battery)
    except Exception as e:
        print('Error: could not load data from file.')
        print(e)
        sys.exit(1)

    # then load the latency data for each file
    data = {}
    for filename in files.keys():
        data[filename] = load_file(filename)

    # make some colors for the different controllers
    colors = ['C{}'.format(i) for i in range(100)]

    # now we have all the data, we can plot it
    fig, ax = plt.subplots()
    # make the y axis values (battery life) start from 5, go to 50, and have a step of 10
    ax.set_yticks(range(10, 50, 10))

    for filename, (controller, battery) in files.items():
        ax.boxplot(data[filename][:,1], positions=[float(battery)], patch_artist=True, vert=0, widths=1, notch=True, showfliers=False, boxprops=dict(facecolor=colors.pop()))

    from matplotlib import ticker
    # Set x tick locations to span x axis, with step=5
    ax.yaxis.set_major_locator(ticker.MultipleLocator(5))
    # set the y axis min and max
    ax.set_ylim(5, 45)
    #Apply default tick formatting so the tick labels all show
    ax.yaxis.set_major_formatter(ticker.ScalarFormatter())

    # add dashed grid lines
    ax.grid(axis='x', linestyle='--')
    ax.grid(axis='y', linestyle='--')

    plt.title(args.title)
    plt.xlabel('Latency (ms)')
    plt.ylabel('Battery Life (hours)')
    # make sure the legend is outside the plot
    plt.legend([controller for filename, (controller, battery) in files.items()], loc='center left', bbox_to_anchor=(1, 0.5))
    # ensure the plot window is big enough to show the legend
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
