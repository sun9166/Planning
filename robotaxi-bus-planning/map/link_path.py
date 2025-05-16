#!/usr/bin/python3
import sys, os, re
import datetime, math



if __name__=="__main__":
    start_x = 0.0
    start_y = -10.0
    
    stop_x = 15.0
    stop_y = -10.0

    if (start_x < stop_x):
        flag_x = 1
    else:
        flag_x = -1

    if (start_y < stop_y):
        flag_y = -1
    else:
        flag_y = 1

    step = 0.1

    cur_x = start_x
    cur_y = start_y

    while (((stop_x - cur_x) * flag_x) >= (step * 2)):
        print(('<point x="%f" y="%f" z="%f"/>') % (cur_x, cur_y, 0.0))
        cur_x += step * flag_x
        cur_y += step * (stop_y - start_y) / (stop_x - start_x) * flag_y
