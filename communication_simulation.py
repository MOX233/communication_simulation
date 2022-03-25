from __future__ import absolute_import
from __future__ import print_function
from math import log2
import random
import numpy as np

class Simulator():
    def __init__(self, W_up, W_down, model_size, Lambda, T_rd, T_cp, dt, T_total, car_speed, road_len, P,N0):
        """"
        args: 
            W_up: bandwidth of uplink (in bit/s)
            W_down: bandwidth of downlink (in bit/s)
            model_size: model size (in bit/model)
            Lambda: arrival rate of cars
            T_rd: round duration
            T_cp: computation delay of cars
            dt: simulation time slot length
            T_total: total simulation time
        """
        self.W_up, self.W_down, self.model_size, self.Lambda, self.T_rd, self.T_cp = W_up, W_down, model_size, Lambda, T_rd, T_cp
        self.T_total = T_total
        self.dt = dt
        self.car_speed = car_speed
        self.road_len = road_len
        self.P = P
        self.N0 = N0
        #self.V_down = W_down/model_size   # in model/s
        #self.V_up = W_up/model_size       # in model/s
        
        self.car_state_list = []
        # car_state: (state, position, progress)
        # state: 0-download, 1-computing, 2-upload, 3-complete, 4-out_of_range
        # position in [0,road_len)
        # progress in [0,1)

        self.round = 0
        self.car_success_dict = {0:0}

    def run_simulation(self):
        time_steps = np.arange(0,self.T_total,self.dt)
        
        for t in time_steps:
            W_down_allocate, W_up_allocate = self.allocate_bandwidth(strategy='uniform_allocate')
            self.car_state_update(W_down_allocate, W_up_allocate)
            self.poisson_arrive()
            if t >= (self.round+1)*self.T_rd:
                self.simulation_state_update(new_round=True)
            else:
                self.simulation_state_update(new_round=False)

    def poisson_arrive(self):
        if np.random.uniform(0,1) < self.Lambda*self.dt:
            self.car_state_list.append([0,0,0])
    
    def allocate_bandwidth(self, strategy):
        if strategy=='uniform_allocate':
            return self.uniform_allocate()
        # need to add in the future

    def car_state_update(self, W_down_allocate, W_up_allocate):
        for car_idx, car_state in enumerate(self.car_state_list):
            car_state[1] += self.car_speed*self.dt
            if car_state[1]>=self.road_len:    #out of range
                self.car_state_list[car_idx] = [4,None,None]
                continue
            if car_state[0]==0:
                W_i = W_down_allocate[car_idx]
                v_i = W_i*log2(1+self.P/self.N0/W_i) # Shannon theorem
                car_state[2] += v_i/self.model_size
                if car_state[2]>=1:
                    self.car_state_list[car_idx] = [1,car_state[1],0]
                else:
                    self.car_state_list[car_idx] = car_state
            elif car_state[0]==1:
                car_state[2] += self.dt/self.T_cp
                if car_state[2]>=1:
                    self.car_state_list[car_idx] = [2,car_state[1],0]
                else:
                    self.car_state_list[car_idx] = car_state
            elif car_state[0]==2:
                W_i = W_up_allocate[car_idx]
                v_i = W_i*log2(1+self.P/self.N0/W_i) # Shannon theorem
                car_state[2] += v_i/self.model_size
                if car_state[2]>=1:
                    self.car_state_list[car_idx] = [3,car_state[1],None]
                    self.car_success_dict[self.round] += 1
                else:
                    self.car_state_list[car_idx] = car_state
        
        #TODO:remove cars which are out of range
        next_car_state_list = []
        for car_idx, car_state in enumerate(self.car_state_list):
            if car_state[0]!=4:
                next_car_state_list.append(car_state)
        self.car_state_list = next_car_state_list
        

    def simulation_state_update(self, new_round):
        if new_round==True:
            self.round += 1
            self.car_success_dict[self.round]=0
            for car_idx, car_state in enumerate(self.car_state_list):
                self.car_state_list[car_idx] = [0,car_state[1],0]

    def uniform_allocate(self):
        W_down_allocate, W_up_allocate = {},{}
        for car_idx, car_state in enumerate(self.car_state_list):
            if car_state[0]==0:
                W_down_allocate[car_idx]=None
            elif car_state[0]==2:
                W_up_allocate[car_idx]=None
        for k in W_down_allocate.keys():
            W_down_allocate[k] = 1./len(W_down_allocate)
        for k in W_up_allocate.keys():
            W_up_allocate[k] = 1./len(W_up_allocate)
        return W_down_allocate, W_up_allocate

    def print_car_success(self):
        for k,v in self.car_success_dict.items():
            print('round {}: {}'.format(k,v))

    def get_statistics(self, prt=False):
        suc_nums = np.asarray(list(self.car_success_dict.values()))
        eps = 1e-12
        avg_suc = np.mean(suc_nums)
        var_suc = np.var(suc_nums)
        C2_suc = var_suc/(avg_suc+eps)**2
        if prt==True:
            print("avg_suc:{:.4f}, var_suc:{:.4f}, C2_suc:{:.4f}".format(avg_suc, var_suc, C2_suc))
        return avg_suc, var_suc, C2_suc
    
    def get_valid_round_ratio(self, prt=False):
        # a round called valid means that during this round, some cars have uploaded weights successfully
        suc_nums = np.asarray(list(self.car_success_dict.values()))
        valid_round_ratio = (len(suc_nums)-np.sum(suc_nums==0))/len(suc_nums)
        if prt==True:
            print("valid_round_ratio:{:.4f}".format(valid_round_ratio))
        return valid_round_ratio
    
    def get_valid_round_num(self, prt=False):
        # a round called valid means that during this round, some cars have uploaded weights successfully
        suc_nums = np.asarray(list(self.car_success_dict.values()))
        valid_round_num = np.sum(suc_nums>0)
        if prt==True:
            print("valid_round_num:{:.4f}".format(valid_round_num))
        return valid_round_num

"""for T_rd in [5,10,15,20,30,50,100,200,500,1000]:
    print('T_rd:',T_rd,'',end='')
    random.seed(1)
    np.random.seed(1)
    sim = Simulator(T_total=10000, dt=0.005,
                 Lambda=0.1, car_speed=20, road_len=400,
                 model_size=100,P=10,N0=1, W_down=1, W_up=1, 
                 T_rd=T_rd, T_cp=5)
    sim.run_simulation()
    sim.get_statistics(prt=True)"""
for T_rd in np.linspace(13,17,12):
    print('T_rd:',T_rd,'')
    random.seed(1)
    np.random.seed(1)
    sim = Simulator(T_total=10000, dt=0.005,
                 Lambda=0.1, car_speed=20, road_len=400,
                 model_size=200,P=1,N0=1, W_down=1, W_up=1, 
                 T_rd=T_rd, T_cp=10)
    sim.run_simulation()
    sim.get_statistics(prt=True)
    sim.get_valid_round_ratio(prt=True)
    sim.get_valid_round_num(prt=True)

