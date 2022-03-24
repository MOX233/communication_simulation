import os
import sys
import random
import csv

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa
def generate_routefile(args):
    random.seed(args.seed)  # make tests reproducible
    
    num_steps = args.num_steps  # number of time steps
    step_length = args.step_length # sumo sampling interval
    Lambda = args.Lambda       # arrival rate of car flow
    accel = args.accel         # accelerate of car flow
    decel = args.decel         # decelerate of car flow
    sigma = args.sigma         # imperfection of drivers, which takes value on [0,1], with 0 meaning perfection and 1 meaning imperfection
    carLength = args.carLength # length of cars
    minGap = args.minGap       # minimum interval between adjacent cars
    maxSpeed = args.maxSpeed   # maxSpeed for cars
    speedFactoer_mean = args.speedFactoer_mean 
    speedFactoer_dev = args.speedFactoer_dev
    speedFactoer_min = args.speedFactoer_min
    speedFactoer_max = args.speedFactoer_max
    
    speedFactoer = "normc({mean}, {dev}, {min}, {max})".format(**{
        "mean":speedFactoer_mean,
        "dev":speedFactoer_dev,
        "min":speedFactoer_min,
        "max":speedFactoer_max,
    }) # can be given as "norm(mean, dev)" or "normc(mean, dev, min, max)"


    with open("sumo_data/road.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typecar" accel="{accel}" decel="{decel}" sigma="{sigma}" length="{carLength}" minGap="{minGap}" maxSpeed="{maxSpeed}" speedFactoer="{speedFactoer}" guiShape="passenger"/>
        <route id="right" edges="right1 right12 right2" />
        <route id="left" edges="left2 left21 left1" />""".format(**{
            "accel":accel,
            "decel":decel,
            "sigma":sigma,
            "carLength":carLength,
            "minGap":minGap,
            "maxSpeed":maxSpeed,
            "speedFactoer":speedFactoer,
        }), file=routes)
        vehNr = 0
        for i in range(int(num_steps/step_length)):
            # just right traffic
            if random.uniform(0, 1) < Lambda*step_length:
                print('    <vehicle id="car_%i" type="typecar" route="right" depart="%.2f" />' % (
                    vehNr, (i*step_length)), file=routes)
                vehNr += 1
        print("</routes>", file=routes)


def sumo_run(options):
    """execute the TraCI control loop"""
    step = 0
    w = csv.writer(open(options.trajectoryInfo_path, 'w',newline=""))
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        for veh_i, veh in enumerate(traci.vehicle.getIDList()):
            (x, y),speed, = [f(veh) for f in [
                traci.vehicle.getPosition,
                traci.vehicle.getSpeed, #Returns the speed of the named vehicle within the last step [m/s]; error value: -1001

            ]]
            w.writerow([step,veh,veh_i,x,speed])
    traci.close()
    sys.stdout.flush()