from __future__ import absolute_import
from __future__ import print_function
from math import log2
import random
import sys
import time
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

    def run_simulation(self,strategy='equal_allocate'):
        time_steps = np.arange(0,self.T_total,self.dt)
        
        scale = 50
        start = time.perf_counter()

        for i,t in enumerate(time_steps):
            a = "*" * int(i/len(time_steps)*scale)
            b = "." * (scale - int(i/len(time_steps)*scale))
            c = (i / len(time_steps)) * 100
            dur = time.perf_counter() - start
            print("\r{:.0f}/{:.0f} {:^3.2f}%[{}->{}]{:.2f}s".format(t,self.T_total,c,a,b,dur),end = "")
            W_down_allocate, W_up_allocate = self.allocate_bandwidth(strategy=strategy)
            self.car_state_update(W_down_allocate, W_up_allocate)
            self.poisson_arrive()
            if t >= (self.round+1)*self.T_rd:
                self.simulation_state_update(new_round=True)
            else:
                self.simulation_state_update(new_round=False)

    def poisson_arrive(self):
        if np.random.uniform(0,1) < self.Lambda*self.dt:
            self.car_state_list.append([0,0,0])

    def car_state_update(self, W_down_allocate, W_up_allocate,eps=1e-8):
        for car_idx, car_state in enumerate(self.car_state_list):
            car_state[1] += self.car_speed*self.dt
            if car_state[1]>=self.road_len:    #out of range
                self.car_state_list[car_idx] = [4,None,None]
                continue
            if car_state[0]==0:
                W_i = W_down_allocate[car_idx]+eps
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
                W_i = W_up_allocate[car_idx]+eps
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
    
    def allocate_bandwidth(self, strategy):
        if strategy=='equal_allocate':
            return self.equal_allocate()
        elif strategy=='random_allocate':
            return self.random_allocate()
        elif strategy=='random_one_hot_allocate':
            return self.random_one_hot_allocate()
        else:
            exit('strategy error!')
        # need to add in the future

    def equal_allocate(self):
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

    def random_one_hot_allocate(self):
        W_down_allocate, W_up_allocate = {},{}
        for car_idx, car_state in enumerate(self.car_state_list):
            if car_state[0]==0:
                W_down_allocate[car_idx]=0
            elif car_state[0]==2:
                W_up_allocate[car_idx]=0
        if len(W_down_allocate)>0:
            one_hot_idx = np.random.randint(len(W_down_allocate))
            for idx,k in enumerate(W_down_allocate.keys()):
                if idx == one_hot_idx:
                    W_down_allocate[k]=1
        if len(W_up_allocate)>0:
            one_hot_idx = np.random.randint(len(W_up_allocate))
            for idx,k in enumerate(W_up_allocate.keys()):
                if idx == one_hot_idx:
                    W_up_allocate[k]=1
        return W_down_allocate, W_up_allocate

    def random_allocate(self):
        W_down_allocate, W_up_allocate = {},{}
        for car_idx, car_state in enumerate(self.car_state_list):
            if car_state[0]==0:
                W_down_allocate[car_idx]=None
            elif car_state[0]==2:
                W_up_allocate[car_idx]=None
        if len(W_down_allocate)>0:
            pp_down = list(np.random.rand(len(W_down_allocate)-1))
            pp_down = sorted(pp_down)
            pp_down.insert(0,0.)
            pp_down.append(1.)
            for idx,k in enumerate(W_down_allocate.keys()):
                W_down_allocate[k] = pp_down[idx+1]-pp_down[idx]
        if len(W_up_allocate)>0:
            pp_up = list(np.random.rand(len(W_up_allocate)-1))
            pp_up = sorted(pp_up)
            pp_up.insert(0,0.)
            pp_up.append(1.)
            for idx,k in enumerate(W_up_allocate.keys()):
                W_up_allocate[k] = pp_up[idx+1]-pp_up[idx]
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

if __name__=='__main__':
    for strategy in ['equal_allocate', 'random_one_hot_allocate', 'random_allocate']:
        print('strategy:',strategy,'')
        random.seed(1)
        np.random.seed(1)
        sim = Simulator(T_total=10000, dt=0.005,
                    Lambda=0.1, car_speed=20, road_len=400,
                    model_size=200,P=1,N0=1, W_down=1, W_up=1, 
                    T_rd=15, T_cp=10)
        sim.run_simulation(strategy=strategy)
        sim.get_statistics(prt=True)
        sim.get_valid_round_ratio(prt=True)
        sim.get_valid_round_num(prt=True)

