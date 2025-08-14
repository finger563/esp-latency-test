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
import re

import argparse

def escape_ansi(line):
    ansi_escape = re.compile(r'(?:\x1B[@-_]|[\x80-\x9F])[0-?]*[ -/]*[@-~]')
    return ansi_escape.sub('', line)

def is_line_valid(line):
    num_elems = len(line.split(','))
    # support both:
    # timestamp, latency
    # timestamp, latency, num_missed_inputs
    is_valid_num_elems = num_elems in [2, 3]
    no_invalid_chars = not ':' in line
    return is_valid_num_elems and no_invalid_chars

def load_file(filename):
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
            lines = [escape_ansi(line) for line in lines]
            # find the index of the header line
            header_index = next(i for i, line in enumerate(lines) if line.startswith('%'))
            # skip all lines before the header line
            lines = lines[header_index+1:]
            # list comprehension to remove any lines that aren't valid CSV
            lines = [line for line in lines if is_line_valid(line)]
            data = np.loadtxt(lines, delimiter=',', skiprows=1)
            return data
    except Exception as e:
        print('Error: could not load data from file.')
        print(e)
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description='Analyze latency data.')
    parser.add_argument('filename', type=str, help='The filename of the latency data.')
    parser.add_argument('--output', type=str, help='The filename to save the histogram to.')
    parser.add_argument('--title', type=str, help='The title of the histogram.', default='Latency Histogram')
    parser.add_argument('--range', type=float, nargs=2, help='The range of the histogram.', default=[0, 150])
    args = parser.parse_args()

    # if no filename is provided, print an error and exit
    if not args.filename:
        print('Error: no filename provided.')
        sys.exit(1)

    # load the file, parse all the rows for the files to analyze
    data = load_file(args.filename)

    # Calculate the mean and standard deviation
    median = np.median(data[:,1])
    mean = np.mean(data[:,1])
    std = np.std(data[:,1])

    # Print the results
    print('Median: {:.2f} ms'.format(median))
    print('Mean: {:.2f} ms'.format(mean))
    print('Standard deviation: {:.2f} ms'.format(std))

    # compute the number of actions per minute (average) over the entire
    # dataset. Every two rows is one action (press and release of a button), and
    # the timestamp is in seconds so we divide by 60 to get minutes.
    total_time = (data[-1,0] - data[0,0]) / 60
    total_actions = data.shape[0] / 2
    actions_per_minute = total_actions / total_time

    print('Actions per minute: {:.2f}'.format(actions_per_minute))

    missed_inputs = None
    # if the data has 3 columns, then the third column is the number of missed
    # inputs, which accumulates over time, so the last row is the total number
    if data.shape[1] >= 3:
        missed_inputs = data[-1,2]

    if missed_inputs is not None:
        print('Missed inputs: {}'.format(missed_inputs))

    # Plot the histogram, over the range x = [0, 150]
    plt.hist(data[:,1], bins=100, range=args.range)
    plt.title(args.title)
    plt.xlabel('Latency (ms)')
    plt.ylabel('Count')
    # add median, mean and std to the plot
    plt.axvline(median, color='y', linestyle='dashed', linewidth=1)
    plt.axvline(mean, color='r', linestyle='dashed', linewidth=1)
    plt.axvline(mean + std, color='g', linestyle='dashed', linewidth=1)
    plt.axvline(mean - std, color='g', linestyle='dashed', linewidth=1)
    # add mean, std-deviation, and actions per minute to the plot as a nice text
    # box in the upper right corner
    plot_text = ''
    plot_text += 'Median: {:.2f} ms\n'.format(median)
    plot_text += 'Mean: {:.2f} ms\n'.format(mean)
    plot_text += 'Std: {:.2f} ms\n'.format(std)
    plot_text += 'Actions/min: {:.2f}'.format(actions_per_minute)
    if missed_inputs is not None:
        plot_text += '\nMissed inputs: {}'.format(missed_inputs)
    plt.text(0.95, 0.95,
             plot_text,
             horizontalalignment='right',
             verticalalignment='top',
             transform=plt.gca().transAxes, bbox=dict(facecolor='white', alpha=0.5))

    if args.output:
        plt.savefig(args.output)
    else:
        plt.show()

if __name__ == '__main__':
    main()
