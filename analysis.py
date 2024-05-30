# python file for analyzing the latency data.
# Latency data is a CSV of the form:
#  timestamp, latency (ms)
#
# The file may have other data at the start or end, but the latency data should
# be in the form of a CSV. It will follow a header line, which starts with a '%'
# and contains the column names.

# This script will output the following:
# - The average latency
# - The standard deviation of the latency
# - A histogram of the latency

import sys
import numpy as np
import matplotlib.pyplot as plt

import argparse

def main():
    parser = argparse.ArgumentParser(description='Analyze latency data.')
    parser.add_argument('filename', type=str, help='The filename of the latency data.')
    parser.add_argument('--output', type=str, help='The filename to save the histogram to.')
    parser.add_argument('--title', type=str, help='The title of the histogram.', default='Latency Histogram')
    args = parser.parse_args()

    # if no filename is provided, print an error and exit
    if not args.filename:
        print('Error: no filename provided.')
        sys.exit(1)

    # load the file, skip all the lines before the header line, and then load
    # the rest of the data into a numpy array
    try:
        with open(args.filename, 'r') as f:
            lines = f.readlines()
            for i, line in enumerate(lines):
                if line[0] == '%':
                    break
            data = np.loadtxt(lines[i+1:], delimiter=',', skiprows=1)
    except Exception as e:
        print('Error: could not load data from file.')
        print(e)
        sys.exit(1)

    # Calculate the mean and standard deviation
    mean = np.mean(data[:,1])
    std = np.std(data[:,1])

    print('Mean: {:.2f} ms'.format(mean))
    print('Standard deviation: {:.2f} ms'.format(std))

    # Plot the histogram, over the range x = [0, 150]
    plt.hist(data[:,1], bins=100, range=(0, 150))
    plt.title(args.title)
    plt.xlabel('Latency (ms)')
    plt.ylabel('Count')
    # add mean and std to the plot
    plt.axvline(mean, color='r', linestyle='dashed', linewidth=1)
    plt.axvline(mean + std, color='g', linestyle='dashed', linewidth=1)
    plt.axvline(mean - std, color='g', linestyle='dashed', linewidth=1)
    # add text to the plot, in the middle of the plot
    plt.text(mean, 0, f'mean ({mean:.2f})', rotation=45, verticalalignment='bottom')
    plt.text(mean + std, 0, f'+std ({std:.2f})', rotation=45, verticalalignment='bottom')
    plt.text(mean - std, 0, f'-std ({std:.2f})', rotation=45, verticalalignment='bottom')
    if args.output:
        plt.savefig(args.output)
    else:
        plt.show()

if __name__ == '__main__':
    main()
