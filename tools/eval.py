# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import math
import sys

xind = 1
yind = 3

# xind_res = 3
# yind_res = 11
xind_orig = 1
yind_orig = 3


def read_data(filename):
    with open(filename) as fp:
        data = [[float(s) for s in l.split(" ")]
                for l in fp.readlines()]

    return {d[0]: d[1:] for d in data}


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print "usage eval.py result_file ground_truth_file"
    else:
        fname = sys.argv[1]
        data = read_data(fname)
        fid = sorted(data.keys())

        x = [data[d][xind] for d in fid]
        y = [data[d][yind] for d in fid]

        fname_orig = sys.argv[2]

        data_orig = read_data(fname_orig)
        fid_orig = sorted(data_orig.keys())
        x_orig = [data_orig[d][xind_orig] for d in fid_orig]
        y_orig = [data_orig[d][yind_orig] for d in fid_orig]

        plt.plot(x, y, 'bo', label="div3")  # label="ROS")
        plt.plot(x_orig, y_orig, 'r', label="orig")
        # label="ground_truth")
        # plt.plot(x_orig, y_orig, 'r', label="ground_truth")
        plt.legend()

        common = set(fid) & set(fid_orig)

        se = sum(
            [((data[i][xind] - data_orig[i][xind_orig]) ** 2 +
                (data[i][yind] - data_orig[i][yind_orig]) ** 2) for i in common])
        rmse = math.sqrt(se / float(len(common)))
        print "rmse:%f, %d samples in (%d, %d) keyframes" % (rmse,
                                                             len(common),
                                                             len(fid),
                                                             len(fid_orig))
        plt.show()
