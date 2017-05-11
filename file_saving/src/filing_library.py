#!/usr/bin/env python

import rospy
import numpy as np
from operator import add

# file to save the object poses
file_address = '/home/davoud/catkin_ws/src/file_saving/src/object_poses_file.txt'

# function to return the n last pose records from the file
def tail(f, n, offset=0):
    """Reads n lines from f with an offset of offset lines."""
    avg_line_length = 74
    to_read = n + offset
    while True:
        try:
            f.seek(-(avg_line_length * to_read), 2)
        except IOError:
            # apparently file is smaller than what we want
            # to step back, go to the beginning instead
            f.seek(0)
        pos = f.tell()
        lines = f.read().splitlines()
        if len(lines) >= to_read or pos == 0:
            return lines[-to_read:offset and -offset or None]
        avg_line_length *= 1.3

# function to read the last n pose records and return the average pose
def avg_last_poses(n = 10):
	global file_address
	f = open(file_address, 'r')
	n_records = tail(f, n)
	avg = [0, 0, 0]
	for i in range(n):
		current_record = n_records[i]
		record_splited = current_record[1:-1].split(',')
		finalized_record = [float(i) for i in record_splited]
		avg = map(add, avg, finalized_record)
	final_avg = [i/float(n) for i in avg]
	print final_avg


	
	



