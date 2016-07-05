#!/bin/bash
clear
./z_pose &
./simOF &
./xy_pose &
rqt_plot
