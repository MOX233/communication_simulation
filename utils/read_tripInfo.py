#!/usr/bin/env python
import csv
from decimal import Decimal

def read_tripInfo(tripInfo_path = 'tripinfo.xml'):
    pass
    car_tripinfo = []
    with open(tripInfo_path, 'r') as f:
        tripinfo = f.readlines()
        for line in tripinfo:
            if line.startswith('    <tripinfo'):
                car_info = line[14:-3].split(" ")
                car_dict = dict()
                for item in car_info:
                    key,value = item.split('=')
                    car_dict[key] = value[1:-1]
                car_tripinfo.append(car_dict)
    return car_tripinfo

def read_trajectoryInfo(args):
    r = csv.reader(open(args.trajectoryInfo_path,'r'))
    car_trajectory = {}
    for row in r:
        [step,veh,veh_i,x,speed] = row
        step,x,speed = int(step),float(x),float(speed)
        timeslot = float(step*Decimal(str(args.step_length)))
        if not veh in car_trajectory:
            car_trajectory[veh] = []
            car_trajectory[veh].append({
                'timeslot':timeslot,
                'position':x,
                'speed':speed,
            })
        else:
            car_trajectory[veh].append({
                'timeslot':timeslot,
                'position':x,
                'speed':speed,
            })
    return car_trajectory