from serial import *

import time
import sys
import os
import os.path
import readline

from threading import Thread


from kinetic import *
from geo import *
from com import *
from visu import *
import upg_kinematic as upg_k
import upg_planning as upg_p




FRONT_LEFT=0
MOVE_UP_ZDELTA=0.2

orders=[None,None,None,None]


#x=0.597

#def sign(i):
#    if i<0:
#        return -1.0
#    return 1.0

def compute_moves(dist,z,angle,store=None):
    """
    Compute the 9 possible positions for verins, adding them in 'store' (that is basically 'orders')

    :param dist: distance between the origin of the robot and the end of legs in the horizontal plan
    :param z: height of the robot platform
    :param angle: angle of the walk direction asked
    :param store: the array where will be stocked all these positions
    :return:
    """
    global FRONT_LEFT

    if store==None:
        store=orders

    while angle<-math.pi:
        angle+=2*math.pi
    while angle>math.pi:
        angle-=2*math.pi
    if angle>=(-math.pi/4.0) and angle<=(math.pi/4.0):
        FRONT_LEFT=0
    elif angle>=(math.pi/4.0) and angle<=(3.0*math.pi/4.0):
        FRONT_LEFT=1
    elif angle<=(-math.pi/4.0) and angle>=(-3.0*math.pi/4.0):
        FRONT_LEFT=3
    else:
        FRONT_LEFT=2

    print("front left is ",FRONT_LEFT)

    start_points=[None,None,None,None]
    for L in [FL,FR,RL,RR]:
        start_points[L]=list(prod(normed([LEGS[L]['origin'][X],LEGS[L]['origin'][Y]]),dist))+[z] # in robot ref
        if inverse_kinetic_robot_ref(LEGS,L,start_points[L])[0]==True:
            print("start point is not in possible space!: ",L," ",start_points[L],' origin:',LEGS[L]['origin'])
            return

    direction=normed([round(math.sin(angle),10),round(math.cos(angle),10),0]) # vector giving direction of the walk

    # start from start_points to find ymax using dichotomic
    ymin=0.1
    ymax=2
    while (ymax-ymin)>0.01:
        y=(ymin+ymax)/2.0
        e=False
        for L in [FL,FR,RL,RR]:
            e= e or inverse_kinetic_robot_ref(LEGS,L,list(plus(start_points[L],prod(direction,y))))[0]
        if e==False:
            ymin=y
        else:
            ymax=y

    # start from start_points to find ymin using dichotomic
    yforward=ymin
    ymin=-0.1
    ymax=-2
    while math.fabs(ymax-ymin)>0.01:
        y=(ymin+ymax)/2.0
        e=False
        for L in [FL,FR,RL,RR]:
            e=e or inverse_kinetic_robot_ref(LEGS,L,list(plus(start_points[L],prod(direction,y))))[0]
        if e==False:
            ymin=y
        else:
            ymax=y
    ybackward=ymin

    # now we know that each leg can move on the line (same z height)
    # from (start_point+direction*ybackward) to (start_point) to (start_point+direction*yforward)


    # record actuator order for specific positions:
    # [0]: backward on the ground
    # [1]: middle   on the ground
    # [2]: forward  on the ground
    # [3]: backward on the air
    # [4]: middle   on the air
    # [5]: forward  on the air
    # [6]: backward slighty up
    # [7]: middle   slighty up
    # [8]: forward  slighty up
    print("angle     : ",angle)
    print("dir       : ",direction)
    print("dir back  : ",ybackward)
    print("dir front : ",yforward)

    for L in [FL,FR,RR,RL]:
        store[L]=[]
        print(L,"back  start: ",start_points[L] , "dir:", prod(direction,ybackward)," => ",plus(start_points[L],prod(direction,ybackward)))
        print(L,"front start: ",start_points[L] , "dir:", prod(direction,yforward)," => ",plus(start_points[L],prod(direction,yforward)))
        store[L].append(inverse_kinetic_robot_ref(LEGS,L,list(plus(start_points[L],prod(direction,ybackward))))[1])
        store[L].append(inverse_kinetic_robot_ref(LEGS,L,list(plus(start_points[L],prod(direction,(ybackward+yforward)/2.0))))[1])
        store[L].append(inverse_kinetic_robot_ref(LEGS,L,list(plus(start_points[L],prod(direction,yforward))))[1])
        p=list(plus(start_points[L],prod(direction,ybackward)))
        p[2]=z+MOVE_UP_ZDELTA
        store[L].append(inverse_kinetic_robot_ref(LEGS,L,p)[1])
        p=list(plus(start_points[L],prod(direction,(ybackward+yforward)/2.0)))
        p[2]=z+MOVE_UP_ZDELTA
        store[L].append(inverse_kinetic_robot_ref(LEGS,L,p)[1])
        p=list(plus(start_points[L],prod(direction,yforward)))
        p[2]=z+MOVE_UP_ZDELTA
        store[L].append(inverse_kinetic_robot_ref(LEGS,L,p)[1])

        p=list(plus(start_points[L],prod(direction,ybackward)))
        p[2]=z+0.05
        store[L].append(inverse_kinetic_robot_ref(LEGS,L,p)[1])
        p=list(plus(start_points[L],prod(direction,(ybackward+yforward)/2.0)))
        p[2]=z+0.05
        store[L].append(inverse_kinetic_robot_ref(LEGS,L,p)[1])
        p=list(plus(start_points[L],prod(direction,yforward)))
        p[2]=z+0.05
        store[L].append(inverse_kinetic_robot_ref(LEGS,L,p)[1])
        print("order for leg:",L)
        for i in range(len(store[L])):
            print('[',i,']',store[L][i],' ',L," ",to_linear_actuator_order(store[L][i]))


######################################################################################

x=1.55
z=-0.65
angle=0
compute_moves(x,z,angle)








def init_walk():
    """
    place legs correctly to begin the walk :
    FL and RR on the 'middle on the ground' position and others on the 'backward on the ground'
    :return:
    """
    tell_controler((FRONT_LEFT+0)%4,to_linear_actuator_order(orders[(FRONT_LEFT+0)%4][1]))
    tell_controler((FRONT_LEFT+1)%4,to_linear_actuator_order(orders[(FRONT_LEFT+1)%4][0]))
    tell_controler((FRONT_LEFT+2)%4,to_linear_actuator_order(orders[(FRONT_LEFT+2)%4][0]))
    tell_controler((FRONT_LEFT+3)%4,to_linear_actuator_order(orders[(FRONT_LEFT+3)%4][1]))



def move_leg(l):
    """
    Move forward the leg from 'backward on the ground' to 'frontward on the ground'
    passing by 'backward on the air' and 'frontward on the air'

    :param l:
    :return:
    """
    tell_controler(l,to_linear_actuator_order(orders[l][3]))
    wait_move(l,5)
    tell_controler(l,to_linear_actuator_order(orders[l][5]))
    wait_move(l,5)
    tell_controler(l,to_linear_actuator_order(orders[l][2]))
    #wait_move(l,5)
    #wait_move(l,1)


pause_term=False

def move_legs(l):
    """
    Move l leg forward, changing the position of the gravity center moving slightly up and down the adjacent leg

    :param l: leg id of the moving leg
    :return:
    """
    tell_controler((FRONT_LEFT+l+2)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l+2)%4][7]))
#    pause_term and raw_input("-")
    move_leg((FRONT_LEFT+l)%4)
#    pause_term and raw_input("-")
    tell_controler((FRONT_LEFT+l+2)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l+2)%4][1]))
    #wait_move((FRONT_LEFT+l+2)%4,1)
    #stop_all_actuators()

# def move():
#     """
#     deprecated walk, replaced by 'Walk' class
#     :return:
#     """
#     init_walk()
#     wait_move(list(range(4)),5)
# #    stop_all_actuators()
#     print("init done, moving leg %d (back right)"%((FRONT_LEFT+2)%4))
#     pause_term and input("-")
#
#     move_legs(2)
#
#     print("moving leg %d (front right)"%((FRONT_LEFT+1)%4))
#     pause_term and input("-")
#
#     move_legs(1)
#
#     print("move done, moving body")
#     pause_term and input("-")
#
#     l=(FRONT_LEFT  )%4
#     tell_controler(l,to_linear_actuator_order(orders[l][0]))
#     l=(FRONT_LEFT+1)%4
#     tell_controler(l,to_linear_actuator_order(orders[l][1]))
#     l=(FRONT_LEFT+2)%4
#     tell_controler(l,to_linear_actuator_order(orders[l][1]))
#     l=(FRONT_LEFT+3)%4
#     tell_controler(l,to_linear_actuator_order(orders[l][0]))
#
#     time.sleep(2)
#     wait_move(list(range(4)),5)
#
#     print("moving leg %d (back left)"%((FRONT_LEFT+3)%4))
#     pause_term and input("-")
#
#     move_legs(3)
#
#     print("moving leg %d (front left)"%((FRONT_LEFT)%4))
#     pause_term and input("-")
#
#
#     move_legs(0)
#
#     print("move done, moving body")
#     pause_term and input("-")
#
#     l=(FRONT_LEFT  )%4
#     tell_controler(l,to_linear_actuator_order(orders[l][1]))
#     l=(FRONT_LEFT+1)%4
#     tell_controler(l,to_linear_actuator_order(orders[l][0]))
#     l=(FRONT_LEFT+2)%4
#     tell_controler(l,to_linear_actuator_order(orders[l][0]))
#     l=(FRONT_LEFT+3)%4
#     tell_controler(l,to_linear_actuator_order(orders[l][1]))
#
#     time.sleep(2)
#     wait_move(list(range(4)),5)
#
#     print("done!")
#     pause_term and input("-")


# def fast_walk():
#     """
#     deprecated fast walk, replaced by FastWalk class
#     """
#     l=(FRONT_LEFT  )%4
#     tell_controler(l,to_linear_actuator_order(orders[l][1]))
#     l=(FRONT_LEFT+1)%4
#     tell_controler(l,to_linear_actuator_order(orders[l][1]))
#     l=(FRONT_LEFT+2)%4
#     tell_controler(l,to_linear_actuator_order(orders[l][1]))
#     l=(FRONT_LEFT+3)%4
#     tell_controler(l,to_linear_actuator_order(orders[l][1]))
#
#     pause_term and input("-")
#
#     l=0 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][4]))
#     l=2 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][4]))
#
#     pause_term and input("-")
#
#     l=0 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][5]))
#     l=2 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][5]))
#     l=1 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][0]))
#     l=3 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][0]))
#
#     wait_move((0,2),2)
#     pause_term and input("-")
#
#     l=0 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][2]))
#     l=2 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][2]))
#     wait_move((0,2),2)
#
#     pause_term and input("-")
#
#     l=1 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][3]))
#     l=3 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][3]))
#
#     pause_term and input("-")
#
#     l=1 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][5]))
#     l=3 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][5]))
#     l=0 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][0]))
#     l=2 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][0]))
#
#     pause_term and input("-")
#
#     l=1 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][2]))
#     l=3 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][2]))


def print_leg_coords():
    """
    display the actual position of the end of the leg
    :return:
    """
    for l in range(4):
        print(l,":",robot_ref_leg_point(LEGS,l,(controlers[l].la[1]['position']+450.0)/1000.0,
                                        (controlers[l].la[0]['position']+450.0)/1000.0,
                                        (controlers[l].la[2]['position']+450.0)/1000.0))

def move_body(dx,dy):
    """
    Deprecated ?
    Move the robot's body of dx, dy

    :param dx: delta x movement asked
    :param dy: delta y movement asked
    :return:
    """
    nord=[]
    for l in range(4):
        print("leg:", l," actuators: ",[d['position'] for d in controlers[l].la])
        print("order supposed to be: ",orders[l][1]," => ",to_linear_actuator_order(orders[l][1]))
        current=robot_ref_leg_point(LEGS,l,(controlers[l].la[1]['position']+450.0)/1000.0,
                                    (controlers[l].la[2]['position']+450.0)/1000.0,
                                    (controlers[l].la[0]['position']+450.0)/1000.0)
        print("leg is at ",current)
        new_point=[current[0]+dx,current[1]+dy,current[2]]
        err,order=inverse_kinetic_robot_ref(LEGS,l,new_point)
        print("new position: ",new_point," leads to ",err," : ",order," => ",to_linear_actuator_order(order))
        if err:
            print("can not make that move: ",current," => ",new_point)
            return
        nord.append(order)
    for l in range(4):
        tell_controler(l,to_linear_actuator_order(nord[l]))
    wait_move(list(range(4)),5)


def move_danse(dx,step,delay,count=1):
    orig_position=[0,0,0,0]
    backup_order=['','','','','']
    for l in range(4):
        print("leg:", l," actuators: ",[d['position'] for d in controlers[l].la])
        backup_order[l]=to_linear_actuator_order(orders[l][1])
        print("order supposed to be: ",orders[l][1]," => ",backup_order[l])
        current=robot_ref_leg_point(LEGS,l,(controlers[l].la[1]['position']+450.0)/1000.0,
                                    (controlers[l].la[2]['position']+450.0)/1000.0,
                                    (controlers[l].la[0]['position']+450.0)/1000.0)
        print("leg is at ",current)
        orig_position[l]=current

    angle=0
    step=math.radians(step)
    c=0
    first=True
    while c<count:
        new_order=[]
        for l in range(4):
            new_point=[orig_position[l][0]+dx*math.cos(angle),orig_position[l][1]+dx*math.sin(angle),orig_position[l][2]]
            err,order=inverse_kinetic_robot_ref(LEGS,l,new_point)
            print("new position: ",new_point," leads to ",err," : ",order," => ",to_linear_actuator_order(order))
            if err:
                print("can not make that move: ",current," => ",new_point)
                return
            new_order.append(to_linear_actuator_order(order))
        for l in range(4):
            tell_controler(l,new_order[l])
        if first:
            first=False
            wait_move(list(range(4)),4)
        else:
            time.sleep(delay)
        angle+=step
        if angle>2*math.pi:
            angle-=2*math.pi
            c+=1


    for l in range(4):
        tell_controler(l,backup_order[l])
    wait_move(list(range(4)),4)



#stop_all_actuators()

print("load visu")
visu=Visu(controlers)
print("run visu")
visu.start()
print("visu ran")


class FastWalk:
    def __init__(self):
        self.stage=0
        self.last=time.time()
        self.stages=[self.setup_init,self.wait,
                     self.setup_legs,self.wait,self.setup_move_legs1,self.wait,self.setup_move_legs2,self.wait,
                     self.move_body1,self.wait,
                     self.move_legs13_1,self.wait,self.move_legs13_2,self.wait,self.move_legs13_3,self.wait,
                     self.move_body2,self.wait,
                     self.move_legs02_1,self.wait,self.move_legs02_2,self.wait,self.move_legs02_3,self.wait,
                     self.loop_walk]

    def timer(self,timeout=None):
        if timeout==None:
            self.last=time.time()
            return False
        return (time.time()-self.last)>timeout
    def setup_init(self):
        l=(FRONT_LEFT  )%4 ; tell_controler(l,to_linear_actuator_order(orders[l][1]))
        l=(FRONT_LEFT+1)%4 ; tell_controler(l,to_linear_actuator_order(orders[l][1]))
        l=(FRONT_LEFT+2)%4 ; tell_controler(l,to_linear_actuator_order(orders[l][1]))
        l=(FRONT_LEFT+3)%4 ; tell_controler(l,to_linear_actuator_order(orders[l][1]))
        self.stage+=1
    def wait(self):
        if wait_move(list(range(4)),1) or self.timer(5):
            self.stage+=1
    def setup_legs(self):
        l=0 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][4]))
        l=2 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][4]))
        self.stage+=1
    def setup_move_legs1(self):
        l=0 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][5]))
        l=2 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][5]))
        self.stage+=1
    def setup_move_legs2(self):
        l=0 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][2]))
        l=2 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][2]))
        self.stage+=1
    def move_body1(self):
        l=0 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][1]))
        l=1 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][0]))
        l=2 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][1]))
        l=3 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][0]))
        self.stage+=1
    def move_legs13_1(self):
        l=1 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][3]))
        l=3 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][3]))
        self.stage+=1
    def move_legs13_2(self):
        l=1 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][5]))
        l=3 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][5]))
        self.stage+=1
    def move_legs13_3(self):
        l=1 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][2]))
        l=3 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][2]))
        self.stage+=1
    def move_body2(self):
        l=0 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][0]))
        l=1 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][1]))
        l=2 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][0]))
        l=3 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][1]))
        self.stage+=1
    def move_legs02_1(self):
        l=0 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][3]))
        l=2 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][3]))
        self.stage+=1
    def move_legs02_2(self):
        l=0 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][5]))
        l=2 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][5]))
        self.stage+=1
    def move_legs02_3(self):
        l=0 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][2]))
        l=2 ; tell_controler((FRONT_LEFT+l)%4,to_linear_actuator_order(orders[(FRONT_LEFT+l)%4][2]))
        self.stage+=1
    def loop_walk(self):
        self.stage=self.move_body1
    def __next__(self):
        if type(self.stage)!=int:
            self.stage=self.stages.index(self.stage)
        s=self.stage
        self.stages[self.stage]()
        if s!=self.stage:
            self.timer()
        return True

def upg_init_pos():
    """put all legs in a known initial position"""
    upg_k.init()
    V = upg_k.get_verins_12()
    tell_controlers(V)
    wait_move(list(range(4)), 5)

class Traj:
    def __init__(self):
        self.stage=0
        self.last=time.time()
    def timer(self,timeout=None):
        if timeout==None:
            self.last=time.time()
            return False
        return (time.time()-self.last)>timeout
    def __next__(self):
        s = self.stage
        if self.stage == 0:
            upg_init_pos()
            self.stage = 1
        elif self.stage == 1:
            if wait_move(list(range(4)), 5) or self.timer(10):
                self.stage = 2
        elif self.stage == 2:
            upg_k.upg_init_legs(controlers)
            Ver = upg_k.get_the_traj()
            for i in range(len(Ver)):
                tell_controlers(Ver[i])
                wait_move(list(range(4)), 0.1)
            self.stage=3
        elif self.stage == 3:
            Ver = upg_k.get_the_traj()
            for i in range(len(Ver)):
                tell_controlers(Ver[i])
                wait_move(list(range(4)), 0.1)
            self.stage = 2
        if self.stage != s:
            print("walk reset timer")
            self.timer()
        return True

class InitCOM:
    def __init__(self):
        self.stage=0
        self.last=time.time()
    def timer(self,timeout=None):
        if timeout==None:
            self.last=time.time()
            return False
        return (time.time()-self.last)>timeout
    def __next__(self):
        s = self.stage
        if self.stage == 0:
            upg_init_pos()
            while not self.timer(2):
                upg_k.upg_init_legs(controlers)
            self.stage = 1
        elif self.stage == 1:
            if wait_move(list(range(4)),0.1) or self.timer(10):
                self.stage = 2
        elif self.stage == 2:
            Ver, LO, LOmega = upg_p.init_com()
            for i in range(len(Ver)):
                tell_controlers(Ver[i])
                wait_move(list(range(4)), 0.1)
            self.stage = 3
        elif self.stage == 3:
            self.timer(10000)
        if self.stage != s:
            print("walk reset timer")
            self.timer()
        return True

class PushUp:
    def __init__(self):
        self.stage=0
        self.last=time.time()
    def timer(self,timeout=None):
        if timeout==None:
            self.last=time.time()
            return False
        return (time.time()-self.last)>timeout
    def __next__(self):
        s = self.stage
        if self.stage == 0:
            upg_init_pos()
            while not self.timer(2):
                upg_k.upg_init_legs(controlers)
            self.stage = 1
        elif self.stage == 1:
            if wait_move(list(range(4)),0.1) or self.timer(10):
                self.stage = 2
        elif self.stage == 2:
            Ver = upg_k.shake_dat_ass_rel(20, 200)
            for i in range(len(Ver)):
                tell_controlers(Ver[i])
                wait_move(list(range(4)), 0.1)
            self.stage=3
        elif self.stage == 3:
            Ver = upg_k.shake_dat_ass_rel(20, 200)
            for i in range(len(Ver)):
                tell_controlers(Ver[i])
                wait_move(list(range(4)), 0.1)
            self.stage = 2
        if self.stage != s:
            print("walk reset timer")
            self.timer()
        return True

class OneByOne:
    def __init__(self):
        self.stage = 0
        self.last = time.time()
    def timer(self, timeout=None):
        if timeout == None:
            self.last = time.time()
            return False
        return (time.time() - self.last) > timeout
    def __next__(self):
        s = self.stage
        nb_steps_per_leg = 30
        amp = 200
        if self.stage == 0:
            upg_init_pos()
            while not self.timer(2):
                upg_k.upg_init_legs(controlers)
            self.stage = 1
        elif self.stage == 1:
            if wait_move(list(range(4)), 0.1) or self.timer(10):
                self.stage = 2
        elif self.stage == 2:
            Ver, LO, LOmega = upg_p.init_com()
            print("one_by_one: moving com to its circle - all legs on ground")
            for i in range(len(Ver)):
                tell_controlers(Ver[i])
                wait_move(list(range(4)), 0.1)
            self.stage = 3
        elif self.stage == 3:
            traj_leg = upg_k.traj_abs_sin_1(nb_steps_per_leg, amp, 0)
            LV, LO, LOmega = upg_p.move_abs_leg_autocom(traj_leg, 0)
            print("one_by_one: moving FL leg")
            for i in range(len(LV)):
                tell_controlers(LV[i])
                wait_move(list(range(4)), 0.1)
            self.stage = 4
        elif self.stage == 4:
            traj_leg = upg_k.traj_abs_sin_1(nb_steps_per_leg, amp, 2)
            LV, LO, LOmega = upg_p.move_abs_leg_autocom(traj_leg, 2)
            print("one_by_one: moving RL leg")
            for i in range(len(LV)):
                tell_controlers(LV[i])
                wait_move(list(range(4)), 0.1)
            self.stage = 5
        elif self.stage == 5:
            traj_leg = upg_k.traj_abs_sin_1(nb_steps_per_leg, amp, 3)
            LV, LO, LOmega = upg_p.move_abs_leg_autocom(traj_leg, 3)
            print("one_by_one: moving RR leg")
            for i in range(len(LV)):
                tell_controlers(LV[i])
                wait_move(list(range(4)), 0.1)
            self.stage = 6
        elif self.stage == 6:
            traj_leg = upg_k.traj_abs_sin_1(nb_steps_per_leg, amp, 1)
            LV, LO, LOmega = upg_p.move_abs_leg_autocom(traj_leg, 1)
            print("one_by_one: moving FR leg")
            for i in range(len(LV)):
                tell_controlers(LV[i])
                wait_move(list(range(4)), 0.1)
            self.stage = 3
        if self.stage!=s:
            print("walk reset timer")
            self.timer()
        return True


class Walk:
    def __init__(self):
        self.stage=0
        self.last=time.time()
    def timer(self,timeout=None):
        if timeout==None:
            self.last=time.time()
            return False
        return (time.time()-self.last)>timeout
    def __next__(self):
        s=self.stage
        if self.stage==0:
            init_walk()
            print("walk: init")
            # upg_k.upg_init_legs(controlers)
            self.stage=1
        elif self.stage==1:
            if wait_move(list(range(4)),1) or self.timer(10):
                self.stage=2
        elif self.stage==2:
            move_legs(2)
            print("walk: moving back right leg (back to front)")
            self.stage=3
        elif self.stage==3:
            if wait_move(list(range(4)),1) or self.timer(10):
                self.stage=4
        elif self.stage==4:
            print("walk: moving front right leg")
            move_legs(1)
            self.stage=5
        elif self.stage==5:
            if wait_move(list(range(4)),1) or self.timer(10):
                self.stage=6
        elif self.stage==6:
            print("walk: moving body")
            l=(FRONT_LEFT  )%4
            tell_controler(l,to_linear_actuator_order(orders[l][0]))
            l=(FRONT_LEFT+1)%4
            tell_controler(l,to_linear_actuator_order(orders[l][1]))
            l=(FRONT_LEFT+2)%4
            tell_controler(l,to_linear_actuator_order(orders[l][1]))
            l=(FRONT_LEFT+3)%4
            tell_controler(l,to_linear_actuator_order(orders[l][0]))
            time.sleep(0.5)
            self.stage=7
        elif self.stage==7:
            if wait_move(list(range(4)),1) or self.timer(5):
                self.stage=8
        elif self.stage==8:
            print("walk: moving back left leg")
            move_legs(3)
            self.stage=9
        elif self.stage==9:
            if wait_move(list(range(4)),1) or self.timer(5):
                self.stage=10
        elif self.stage==10:
            print("walk: moving front left leg")
            move_legs(0)
            self.stage=11
        elif self.stage==11:
            if wait_move(list(range(4)),1) or self.timer(5):
                self.stage=12
        elif self.stage==12:
            print("walk: moving body")
            l=(FRONT_LEFT  )%4
            tell_controler(l,to_linear_actuator_order(orders[l][1]))
            l=(FRONT_LEFT+1)%4
            tell_controler(l,to_linear_actuator_order(orders[l][0]))
            l=(FRONT_LEFT+2)%4
            tell_controler(l,to_linear_actuator_order(orders[l][0]))
            l=(FRONT_LEFT+3)%4
            tell_controler(l,to_linear_actuator_order(orders[l][1]))
            self.stage=13
        elif self.stage==13:
            if wait_move(list(range(4)),1) or self.timer(5):
                self.stage=2
        if self.stage!=s:
            print("walk reset timer")
            self.timer()
        return True


class SpiderUp:
    def __init__(self):
        self.starting=True
    def __next__(self):
        if self.starting:
            for i in [0,1,2,3]:
                tell_controler(i,to_linear_actuator_order(orders[i][1]))
            self.starting=False
        else:
            if wait_move(list(range(4)),1,0.1):
                return False
        return True
    
class UpAndDown:
    def __init__(self,zmin,zmax):
        self.waiting=True
        self.minorders=[None,None,None,None]
        compute_moves(x,zmin,angle,self.minorders)
        self.maxorders=[None,None,None,None]
        compute_moves(x,zmax,angle,self.maxorders)
        self.direction=1
    def __next__(self):
        if wait_move(list(range(4)),10):
            if self.direction==1:
                print("move up")
                for i in [0,1,2,3]:
                    tell_controler(i,to_linear_actuator_order(self.maxorders[i][1]))
                time.sleep(1)
                self.direction=-1
            else:
                print("move down")
                for i in [0,1,2,3]:
                    tell_controler(i,to_linear_actuator_order(self.minorders[i][1]))
                time.sleep(1)
                self.direction=1                
    
class Movement(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.current_move=None
        self.suspend=False
        self.stop=False
    def run(self):
        while self.stop==False:            
            if self.suspend or self.current_move==None:
                time.sleep(0.5)
                continue
            if next(self.current_move)==False: # movement is over
                print("current movement is over!")
                self.current_move=None
    def quit(self):
        self.stop=True
    def pause(self):
        self.suspend=True
    def unpause(self):
        self.suspend=False
    def setMove(self,move):
        self.current_move=move

movement_handler=Movement()
movement_handler.start()


cmds={}

def cmd_upgrade(m):
    """upgrade the controlers (optional controler id for single upgrade)"""
    if len(m)>1:
        upgrade_controlers([int(m[1])])
    else:
        upgrade_controlers()

def cmd_help(m):
    """print help"""
    l=list(cmds.keys())
    l.sort()
    for k in l:
        print(k, end=' ')
        if cmds[k].__doc__!=None:
            print(":",cmds[k].__doc__)
        else:
            print()

def cmd_vpause(m):
    """pause/unpause graphic update"""
    visu.pause()
            
def cmd_dst(m):
    """get/set wait move distance in mm"""
    global z,x,angle,WAITMOVE_DST
    if len(m)>1:
        WAITMOVE_DST[0]=float(m[1])
    print("wait move distance is:",WAITMOVE_DST[0])
def cmd_zdelta(m):
    """get/set z difference between ground and air position"""
    global z,x,angle,MOVE_UP_ZDELTA
    if len(m)>1:
        MOVE_UP_ZDELTA=float(m[1])
        compute_moves(x,z,angle)
    print("zdelta for move is ",MOVE_UP_ZDELTA)
def cmd_order(m):
    """send an order to a controler"""
    if len(m[2])==1:
        l=int(m[1])
        tell_controler(l,to_linear_actuator_order(orders[l][int(m[2])]))
    else:
        tell_controler(int(m[1]),m[2]+'\n')
def cmd_rescan(m):
    """rescan usb tty for controlers"""
    find_controler()
def cmd_position(m):
    """move a leg into a precomputed position (0 to 8)"""
    tell_controler(int(m[1]),to_linear_actuator_order(orders[int(m[1])][int(m[2])]))
def cmd_status(m):
    """print actuator status"""
    for c in controlers:
        c.print_actuators_status()
def cmd_visup(m):
    """increase visu time by 1sec"""
    visu.setTimeDisplay(visu.getTimeDisplay()+1)
def cmd_visum(m):
    """decrease visu time by 1sec"""
    visu.setTimeDisplay(max(visu.getTimeDisplay()-1,1))
def cmd_print(m):
    """print legs coordinates"""
    print_leg_coords()
def cmd_body(m):    
    move_body(float(m[1]),float(m[2]))
def cmd_danse(m):
    move_danse(float(m[1]),float(m[2]),float(m[3]),int(m[4]))
def cmd_up(m):
    """raise position by 5cm"""
    global z,x,angle
    z-=0.05
    compute_moves(x,z,angle)
def cmd_down(m):
    """lower position by 5cm"""
    global z,x,angle
    z+=0.05
    compute_moves(x,z,angle)
def cmd_far(m):
    """expend legs by 5cm"""
    global z,x,angle
    x+=0.05
    compute_moves(x,z,angle)
def cmd_close(m):
    """retract legs by 5cm"""
    global z,x,angle    
    x-=0.05
    compute_moves(x,z,angle)
def cmd_angle(m):
    """get/set walk angle"""
    global z,x,angle
    if len(m)>1:
        angle=math.radians(float(m[1]))
        compute_moves(x,z,angle)
    print("angle:",math.degrees(angle))
def cmd_quit(m):
    """leave program"""
    for c in controlers:
        c.stop()
    for c in controlers:
        c.join()
    visu.quit()
    movement_handler.quit()
    movement_handler.join()
    sys.exit(0)
def cmd_traj(m):
    """start making the traj (circle or straight)(i hope)"""
    movement_handler.setMove(Traj())
def cmd_initcom(m):
    """start initializing COM position"""
    movement_handler.setMove(InitCOM())
def cmd_pushup(m):
    """start push-up movement"""
    movement_handler.setMove(PushUp())
def cmd_onebyone(m):
    """start initializing one_by_one movement"""
    movement_handler.setMove(OneByOne())
def cmd_walk(m):
    """start walk movement"""
    movement_handler.setMove(Walk())
def cmd_fast(m):
    """start fast walk movement"""
    movement_handler.setMove(FastWalk())
def cmd_pause(m):
    """toggle pause in motion"""
    global pause_term
    pause_term=not pause_term
    print("pause during moving :",pause_term)
def cmd_u(m):
    """start Spider Up movement"""
    movement_handler.setMove(SpiderUp())
def cmd_ud(m):
    """start an up and down movement, need zmin and zmax"""
    movement_handler.setMove(UpAndDown(float(m[1]),float(m[2])))
        
import inspect
import sys



for f in inspect.getmembers(sys.modules[__name__]):
    if f[0].startswith('cmd_'):
        cmds[f[0][4:]]=f[1]


def readline_completer(text,state):
    l=[]
    if len(text)>0:
        for c in list(cmds.keys()):
            if c.startswith(text):
                l.append(c)
    l.append(None)
    return l[state]




if __name__ == "__main__":

    readline.parse_and_bind('tab: complete')
    readline.parse_and_bind('set editing-mode emacs')
    try:
        readline.read_history_file(".history")
    except:
        pass

    readline.set_completer(readline_completer)

    last_cmd=""
    while True:
        try:
            m=input("\u001b[31m (x=%.2f,z=%.2f/%.2f,angle=%.2f,pause=%s)? \u001b[0m "%(x,z,MOVE_UP_ZDELTA,math.degrees(angle),str(pause_term)))
            #readline.add_history(m)
            readline.write_history_file(".history")
            if len(m)==0:
                m=last_cmd
            last_cmd=m
            m=m.split()
            if m[0] in cmds:
                cmds[m[0]](m)
            else:
                movement_handler.setMove(None)
                stop_all_actuators()
        except Exception as e:
            print(e)
            import traceback
            traceback.print_exc(file=sys.stdout)

