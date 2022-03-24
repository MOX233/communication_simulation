#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Python version: 3.8

import argparse

def args_parser():
    parser = argparse.ArgumentParser()
    # federated arguments
    parser.add_argument('--lr', type=float, default=0.05, help="learning rate")
    parser.add_argument('--momentum', type=float, default=0.5, help="SGD momentum (default: 0.5)")
    parser.add_argument('--seed', type=int, default=1, help='random seed which make tests reproducible (default: 1)')
    
    # SUMO arguments
    parser.add_argument("--nogui", action="store_true",
                         default=True, help="run the commandline version of sumo")
    parser.add_argument("--trajectoryInfo_path", type=str,
                         default='./sumo_result/trajectory.csv', help="the file path where stores the trajectory infomation of cars")
    parser.add_argument("--step_length", type=float, 
                         default=0.1, help="sumo sampling interval")
    parser.add_argument("--num_steps", type=int, 
                         default=10000, help="number of time steps, which means how many seconds the car flow takes")
    parser.add_argument("--round_duration", type=float, 
                         default=100, help="duration time of each round")
    parser.add_argument("--Lambda", type=float, 
                         default=0.1, help="arrival rate of car flow")
    parser.add_argument("--accel", type=float, 
                         default=1000000, help="accelerate of car flow")
    parser.add_argument("--decel", type=float, 
                         default=1000000, help="decelerate of car flow")
    parser.add_argument("--sigma", type=float, 
                         default=0, help="imperfection of drivers, which takes value on [0,1], with 0 meaning perfection and 1 meaning imperfection")
    parser.add_argument("--carLength", type=float, 
                         default=0.01, help="length of cars")
    parser.add_argument("--minGap", type=float, 
                         default=0.01, help="minimum interval between adjacent cars")
    parser.add_argument("--maxSpeed", type=float, 
                         default=20, help="maxSpeed for cars")
    parser.add_argument("--speedFactoer_mean", type=float, 
                         default=1, help="")
    parser.add_argument("--speedFactoer_dev", type=float, 
                         default=0, help="")
    parser.add_argument("--speedFactoer_min", type=float, 
                         default=1, help="")
    parser.add_argument("--speedFactoer_max", type=float, 
                         default=1, help="")
    args = parser.parse_args()
    return args