#!/usr/bin/python
from tkinter import *

import math
import sys
import numpy

X=0
Y=1
Z=2



FL=0 # front left leg
FR=1 # front right leg
RL=2 # rear left leg
RR=3 # rear right leg

TIME_FACTOR=0.5
UPDATE_FREQ=100.0

#ORIGINS={FL:[-0.4,0.4,0],FR:[0.4,0.4,0],RL:[-0.4,-0.4,0],RR:[0.4,-0.4,0]}
#ANGLES={FL:3.0*math.pi/4.0,FR:math.pi/4.0,RL:5.0*math.pi/4.0,RR:7.0*math.pi/4.0}
#ANGLES_ORIG={FL:3.0*math.pi/4.0,FR:math.pi/4.0,RL:5.0*math.pi/4.0,RR:7.0*math.pi/4.0}

SPEED={'v1':80,'v2':40,'v3':20}

#MATRICES={}
#IMATRICES={}

LEGS={FL:{'origin':(-0.4,0.4,0),         \
          'angle':3.0*math.pi/4.0,       \
          'angle_orig':3.0*math.pi/4.0,  \
          'matrix': [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],                  \
          'imatrix':[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]                }, \
      FR:{'origin':(0.4,0.4,0),          \
          'angle':math.pi/4.0,           \
          'angle_orig':math.pi/4.0,      \
          'matrix': [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],                  \
          'imatrix':[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]                }, \
      RL:{'origin':(-0.4,-0.4,0),        \
          'angle':5.0*math.pi/4.0,       \
          'angle_orig': 5.0*math.pi/4.0, \
          'matrix': [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],                  \
          'imatrix':[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]                }, \
      RR:{'origin':(0.4,-0.4,0),         \
          'angle':7.0*math.pi/4.0,       \
          'angle_orig':7.0*math.pi/4.0,  \
          'matrix': [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],                  \
          'imatrix':[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]                }  \
}

ROBOT={'LEGS':LEGS,'SPEED':SPEED}

for l in FL,FR,RL,RR:
    LEGS[l]['v1sim']=550
    LEGS[l]['v2sim']=550
    LEGS[l]['v3sim']=550


LEGS_CANVAS={}
LEGS_SLIDER={FL:{},FR:{},RL:{},RR:{}}    

def update_transformation_matrices_for(matrix,imatrix,angle,origin):
    m = [ [math.cos(angle) , 0 , -math.sin(angle) , origin[X] ],
          [math.sin(angle) , 0 , -math.cos(angle) , origin[Y] ],
          [              0 , 1 ,                0 , origin[Z] ],
          [              0 , 0 ,                0 , 1 ] ]
    
    im = [ [math.cos(-angle) , -math.sin(-angle) , 0 , -origin[X] * math.cos(-angle) + origin[Y] * math.sin(-angle)],
           [               0 ,                0 , 1 , -origin[Z] ],
           [math.sin(angle)  , -math.cos(angle) , 0 , origin[X] * math.sin(-angle) + origin[Y] * math.cos(-angle) ],
           [               0 ,                0 , 0 , 1 ] ]
    for l in range(len(matrix)):
        for c in range(len(matrix[0])):
            matrix[l][c]=m[l][c]
            imatrix[l][c]=im[l][c]
                   



    
# compute transformation matrices
def update_transformation_matrices(legs):
    for l in FL,FR,RL,RR:
        update_transformation_matrices_for(legs[l]['matrix'],legs[l]['imatrix'],legs[l]['angle'],legs[l]['origin'])



update_transformation_matrices(ROBOT['LEGS'])

def draw_points(canvas,points,lines,vectors,msg):
    canvas_width = int(canvas['width'])
    canvas_height = int(canvas['height'])
    shift_x=canvas_width/2
    shift_y=canvas_height/2
    scale=CANVAS_SCALE
    #print (points)
    npoints={}
    for k in points:
        npoints[k]=(points[k][0]*scale+shift_x,-points[k][1]*scale+shift_y)
    for k in npoints:
        p=npoints[k]
        #print p
        canvas.create_oval(p[0],p[1],p[0]+5,p[1]+5,fill="#476042")
        canvas.create_text(p[0]-50,p[1]-15,text=k+'\n(%.2f,%.2f)'%(points[k][0],points[k][1]))
        #canvas.create_text(p[0]-10,p[1]-15,text=k)
    for (a,b,c) in lines:
        #print a,b,c
        canvas.create_line(npoints[a][0],npoints[a][1],npoints[b][0],npoints[b][1],
                      fill=c,width=5)
    #print vectors
    for a in vectors:
        canvas.create_line(npoints[a][0],npoints[a][1],npoints[a][0]+vectors[a][0]/10.0,npoints[a][1]-vectors[a][1]/10.0,
                      fill="magenta",width=5)
    if msg!=None:
        #canvas.create_text(200,200,text=msg,justify=LEFT)
        pass
        


from math import sqrt

def norm2(v):
    n=0
    for x in v:
        n+=x**2
    return n
def norm(v):
    return sqrt(norm2(v))

def normed(v):
    n=norm(v)
    assert(n!=0)
    return tuple([a/n for a in v])

def dist(a,b):
    return norm(((b[0]-a[0]),(b[1]-a[1])))

def p2v(a,b):
    return tuple ([b[i] - a[i] for i in range(len(a))])

def plus(v1,v2):
    return tuple ([ v1[i] + v2[i] for i in range(len(v1)) ])

def prod(v1,a):
    return tuple([i*a for i in v1])
    

def one_line(centre, point , distance):
    cp=p2v(centre,point)
    d=normed(cp)
    return plus(centre,prod(d,distance))

def scalaire(v1,v2):
    n=0
    for i in range(len(v1)):
        n+=v1[i]*v2[i]
    return n

def vectoriel(v1,v2):
    return v1[0]*v2[1] - v1[1]*v2[0]

def vectoriel3D(v1,v2):
    return (
        v1[Y]*v2[Z] - v1[Z]*v2[Y],
        v1[Z]*v2[X] - v1[X]*v2[Z],
        v1[X]*v2[Y] - v1[Y]*v2[X]
    )
def compute(xxx_todo_changeme, xxx_todo_changeme1,da,db):
    (xa,ya) = xxx_todo_changeme
    (xb,yb) = xxx_todo_changeme1
    xa=float(xa)
    ya=float(ya)
    xb=float(xb)
    yb=float(yb)
    da=float(da)
    db=float(db)
    Dab = sqrt((xb-xa)**2 + (yb-ya)**2)
    xp=(da**2-db**2 + Dab**2) / (2.0*Dab)
    t=da**2-xp**2
    if t<0:
        return ()
    if t==0:
        yp1=0
        yp2=0
    else:
        yp1=sqrt(t)
        yp2=-yp1
    i = normed( ((xb-xa), (yb-ya)) )
    x1=xa + xp * i[0]  - yp1*i[1]
    y1=ya + xp * i[1]  + yp1*i[0]
    x2=xa + xp * i[0]  - yp2*i[1]
    y2=ya + xp * i[1]  + yp2*i[0]
    return ( (x1,y1) , (x2,y2) )

def middle(a,b):
    return (a[0]+(b[0]-a[0])/2,a[1]+(b[1]-a[1])/2)

def get_gravity_center(points,weigths):
    mw=[]
    total_weigth=0
    tweigths=[]
    for p1 in range(len(points)):
        np1=list(points.keys())[p1]
        for p2 in range(p1+1,len(points)):
            np2=list(points.keys())[p2]
            if np1 in weigths and np2 in weigths[np1]:
                tweigths.append((np1,np2,weigths[np1][np2]))
    for a,b,w in tweigths:
        m=middle(points[a],points[b])
        mw.append((m,w))
        total_weigth += w
    g=[0,0]
    for m,w in mw:
        g[0]+=m[0]*w
        g[1]+=m[1]*w
    g[0]=g[0]/total_weigth
    g[1]=g[1]/total_weigth
    return (g,total_weigth)
    

#print compute( (1, 1) , (2 , 2) , 1 ,1)

def get_leg_points(V1,V2,AO,BC,AE,DE,EF,FG,FH,GI,BF,GJ):
    #print 'get leg width V1: ',V1,' and V2: ',V2
    points={'O':(0.0,0.0),'A':(-AO,0.0),'B':(0.10,0),'C':(BC+0.1,0.06),'Q':(-0.8,0),'R':(-0.8-0.79,-0.49)}

    points['D'],p2= compute(points['A'],points['C'], AE-DE , V1) 
    points['E'] = one_line(points['A'], points['D'] , AE)
    #print points['D']
    #print points['E']
    #print vectoriel(p2v(points['A'],points['D']) , p2v(points['A'],points['E']))
    #print dist(points['A'],points['E'])
    #print scalaire(p2v(points['A'],points['D']) , p2v(points['A'],points['E']))
    points['F'],p2= compute(points['E'],points['B'], EF , BF)
    points['H'] = one_line(points['B'], points['F'] , BF-FH)
    points['G'] = one_line(points['E'], points['F'] , EF+FG)
    points['I'],p2= compute(points['G'],points['H'], GI , V2)
    points['J'] = one_line(points['G'], points['I'] , GJ)
    return points

def get_leg_points_V1_V2(v1,v2):
    return get_leg_points(v1,
                          v2,
                          int(ao.get())/1000.0,
                          int(bc.get())/1000.0,
                          int(ae.get())/1000.0,
                          int(de.get())/1000.0,
                          int(ef.get())/1000.0,
                          int(fg.get())/1000.0,
                          int(fh.get())/1000.0,
                          int(gi.get())/1000.0,
                          int(bf.get())/1000.0,
                          int(gj.get())/1000.0)

def compute_gravity_stengths_2points(points,gravity,a,b):
    forces={}
    forces['M'] = (0,-gravity[1]*9.81)
    forces[b] = (0,gravity[1]*9.81 * (points['M'][0] - points[a][0]) / (points[b][0] - points[a][0]))
    forces[a] = (0,gravity[1]*9.81 * (points[b][0] - points['M'][0]) / (points[b][0] - points[a][0]))
    return forces

def compute_actuator_strengths_VE(points,weigths,Forces):
    S2points={}
    for p in ['R','Q','O','A','B','C','E','D','F','G','H']:
        S2points[p]=points[p]
    G2=get_gravity_center(S2points,weigths)
    #print "Gravity for S2:",G2
    S3points={}
    for p in ['H','I']:
        S3points[p]=points[p]
    G3=get_gravity_center(S3points,weigths)
    #G3=(G3[0],0.001)
    #print "Gravity for S3:",G3    
    S4points={}
    for p in ['G','I','J']:
        S4points[p]=points[p]
    G4=get_gravity_center(S4points,weigths)
    #print "Gravity for S4:",G4
    total_weight=G2[1]+G3[1]+G4[1]
    #print "MX:",(G2[0][X]*G2[1] + G3[0][X]*G3[1] + G4[0][X]*G4[1]) / total_weight
    #print "MY:",(G2[0][Y]*G2[1] + G3[0][Y]*G3[1] + G4[0][Y]*G4[1]) / total_weight
    #print "W:",total_weight
    
    #print "A :",points['R']
    #print "B :",points['J']
    #print "C :",points['H']
    #print "D :",points['I']
    #print "E :",points['G']
    
    a12=G4[1]*9.81*(G4[0][X]-points['G'][X]) + (points['G'][X] - points['J'][X])*Forces['J'][Y]
    a2 = points['I'][X]-points['H'][X]
    a10 = points['I'][Y]-points['G'][Y]
    a3 = points['H'][Y] - points['I'][Y]
    a4 = G3[1]*9.81*(G3[0][X] - points['H'][X])
    a5 = points['G'][X] - points['I'][X]
    
    FD=[0,0]
    FD[X] = (a12 * a2 - a5  * a4) / (a10 * a2 - a3 * a5 )
    FD[Y] = (a12 * a3 - a10 * a4) / (a5  * a3 - a2 * a10)
    #print "FD:",FD
    FC1=[0,0]
    FC1[X] = - FD[X]
    FC1[Y] = G3[1]*9.81-FD[Y]

    #print "FC1:",FC1
    
    FC2=[0,0]
    a=points['H'][Y] - points['G'][Y]
    b=points['G'][X] - points['H'][X]
    c=G2[1]*9.81*(G2[0][X] - points['G'][X]) + (points['G'][X]-points['R'][X]) * Forces['R'][Y]
    ap=points['I'][Y] - points['H'][Y]
    bp=points['H'][X] - points['I'][X]
    cp=G3[1] * 9.81 * (G3[0][X] - points['I'][X])

    FC2[X] = (cp * b - bp * c) / (ap*b-a*bp)
    FC2[Y] = (cp*a-ap*c) / (bp*a-b*ap)
    #print "FC2:",FC2
    
    Forces['H'] = FC2
    Forces['I'] = FD
    Forces['G'] = [0,0]
    Forces['G'][X] = Forces['I'][X] - Forces['J'][X]
    Forces['G'][Y] = G4[0][1]*9.81 + Forces['I'][Y] - Forces['J'][Y]
    

def compute_actuator_strengths_VM(points,weigths,forces):
    S21points={}
    for p in ['E','F','G']:
        S21points[p]=points[p]
    G21=get_gravity_center(S21points,weigths)
    #print "Gravity for S21:",G21

    S22points={}
    for p in ['A','D','E']:
        S22points[p]=points[p]
    G22=get_gravity_center(S22points,weigths)
    #print "Gravity for S22:",G22
    
    S23points={}
    for p in ['R','A','B','C','O','Q']:
        S23points[p]=points[p]
    G23=get_gravity_center(S23points,weigths)
    #print "Gravity for S23:",G23
    
    S24points={}
    for p in ['D','C']:
        S24points[p]=points[p]
    G24=get_gravity_center(S24points,weigths)
    #print "Gravity for S24:",G24
    
    S25points={}
    for p in ['B','H','F']:
        S25points[p]=points[p]
    G25=get_gravity_center(S25points,weigths)
    #print "Gravity for S25:",G25
    
    S3points={}
    for p in ['H','I']:
        S3points[p]=points[p]
    G3=get_gravity_center(S3points,weigths)
    #G3=(G3[0],0.001)
    #print "Gravity for S3:",G3
    
    S4points={}
    for p in ['G','I','J']:
        S4points[p]=points[p]
    G4=get_gravity_center(S4points,weigths)
    #print "Gravity for S4:",G4
    
    total_weight=G21[1]+G22[1]+G23[1]+G24[1]+G25[1]+G3[1]+G4[1]
    #print "MX:",(G21[0][X]*G21[1] +G22[0][X]*G22[1] + G23[0][X]*G23[1] + G24[0][X]*G24[1] + G25[0][X]*G25[1] + G3[0][X]*G3[1] + G4[0][X]*G4[1]) / total_weight
    #print "MY:",(G21[0][Y]*G21[1] +G22[0][Y]*G22[1] + G23[0][Y]*G23[1] + G24[0][Y]*G24[1] + G25[0][Y]*G25[1] + G3[0][Y]*G3[1] + G4[0][Y]*G4[1]) / total_weight
    #print "W:",total_weight


    a1 = G21[1]*9.81*(G21[0][X] - points['F'][X]) + (points['G'][X] - points['F'][X])*forces['G'][Y] + (points['F'][Y] - points['G'][Y]) * forces['G'][X] \
         +(points['F'][X] - points['E'][X])*(G25[1]*9.81+G21[1]*9.81+forces['G'][Y] + forces['H'][Y]) \
         +(points['E'][Y] - points['F'][Y])*(forces['H'][X] - forces['G'][X])
    a2 = points['E'][Y] - points['F'][Y]
    a3 = points['F'][X] - points['E'][X]
    a4 = points['B'][X] - points['F'][X]
    a5 = points['F'][Y] - points['B'][Y]
    a6 = G25[1] * 9.81 * (G25[0][X] - points['F'][X]) + (points['H'][X] - points['F'][X]) * forces['H'][Y] + (points['F'][Y] - points['H'][Y])*forces['H'][X]

    FB=[0,0]
    FB[X] = (a6 * a3 - a4*a1) / (a3*a5 - a2*a4)
    FB[Y] = -(a2*a6-a1*a5)/(a3*a5-a2*a4)
    forces['B']=FB
    FE=[0,0]
    FE[X] = forces['H'][X] + forces['G'][X] - forces['B'][X]
    FE[Y] = G25[1]*9.81 + G21[1]*9.81 + forces['G'][Y] + forces['H'][Y] - forces['B'][Y]                    
    forces['E']=FE

    b1 = G24[1]*9.81*(G24[0][X] - points['C'][X])
    b2 = points['D'][Y] - points['C'][Y]
    b3 = points['C'][X] - points['D'][X]
    b4 = points['A'][X] - points['C'][X]
    b5 = points['C'][Y] - points['A'][Y]
    b6 = G23[1] * 9.81 * (G23[0][X] - points['A'][X]) + (points['A'][X] - points['R'][X]) * forces['R'][Y] - (points['A'][X] - points['B'][X])*forces['B'][Y] -  (points['B'][Y] - points['A'][Y])*forces['B'][X]

    FC=[0,0]
    FC[X] = (b6 * b3 - b4*b1) / (b3*b5 - b2*b4)
    FC[Y] = -(b2*b6-b1*b5)/(b3*b5-b2*b4)
    forces['C']=FC

    FD=[0,0]
    FD[X] = - forces['C'][X]
    FD[Y] = G24[1]*9.81 - forces['C'][Y]
    forces['D']=FD

    FA=[0,0]
    FA[X] = forces['E'][X] + forces['D'][X]
    FA[Y] = G22[1] * 9.81 + forces['E'][Y] + forces['D'][Y]
    forces['A'] = FA

    FF=[0,0]
    FF[X] = forces['G'][X] - forces['E'][X]
    FF[Y] = G21[1] * 9.81 + forces['G'][Y] - forces['E'][Y]
    forces['F']=FF


def compute_forces_and_stress(points,weights,forces,stress):    
    gravity=get_gravity_center(points,weigths)
    points['M'],weigth=gravity
    f=compute_gravity_stengths_2points(points,gravity,'R','J')
    for i in f:
        forces[i]=f[i]
    compute_actuator_strengths_VE(points,weigths,forces)
    compute_actuator_strengths_VM(points,weigths,forces)
    
    sup=normed(p2v(points['C'],points['D']))
    stress['C']=math.fabs(scalaire(sup,forces['C']))
    stress['D']=math.fabs(scalaire(sup,forces['D']))    
    sup=normed(p2v(points['H'],points['I']))
    stress['I']=math.fabs(scalaire(sup,forces['I']))
    stress['H']=math.fabs(scalaire(sup,forces['H']))
    
#points=get_leg_points(V1,V2)
lines=[('A','B','black'),('A','E','black'),('B','C','black'),('B','F','black'),('E','G','black'),
       ('G','J','black'),('D','C','red'),('H','I','red'),('J1','J2','blue'),('J2','J4','blue'),
       ('J4','J3','blue'),('J3','J1','blue'),('R','Q','black'),('Q','O','black')]

#weigths=[('A','B',2.0),('A','C',1.6),('A','E',3.6),('E','G',4.0),('B','F',3.45),('G','J',3.65),('D','C',5.3),('H','I',5.3),('R','Q',32),('O','Q',1)]
weigths={}
for p in 'ABCDEFGHIJO':
    weigths[p]={}
weigths['Q']={}
weigths['R']={}
weigths['A']['B']=2.0
weigths['A']['C']=1.6
weigths['A']['E']=3.6
weigths['E']['G']=4.0
weigths['B']['F']=3.45
weigths['G']['J']=3.65
weigths['D']['C']=5.3
weigths['H']['I']=5.3
weigths['R']['Q']=32
weigths['O']['Q']=80
tweigths={}
for a in weigths:
    for b in weigths[a]:
        weigths[b][a]=weigths[a][b]


def update_robot(robot=None):
    if robot==None:
        irobot=ROBOT
        ilegs=ROBOT['LEGS']
    else:
        irobot=robot
        ilegs=robot['LEGS']
        
    irobot['feets']={}
    irobot['centers']={}
    for leg in FL,FR,RL,RR:
        if 'points' not in ilegs[leg]:
            continue
        feet=ilegs[leg]['points']['J']
        feet=(feet[0],feet[1],0,1)
        rfeet=numpy.matmul(ilegs[leg]['matrix'],feet).tolist()
        irobot['feets'][leg]=rfeet[:3]
    
        center=ilegs[leg]['points']['O']
        center=(center[0],center[1],0,1)
        rcenter=numpy.matmul(ilegs[leg]['matrix'],center).tolist()

        irobot['centers'][leg] = rcenter[:3]
    
    # compute gravity center of the robot
    irobot['gravities']={}
    for leg in FL,FR,RL,RR:
        if 'gravity' not in ilegs[leg]:
            return
        lg=ilegs[leg]['gravity'][0]
        lg=(lg[0],lg[1],0,1)
        rlg=numpy.matmul(ilegs[leg]['matrix'],lg).tolist()
        irobot['gravities'][leg]=(rlg[:3],ilegs[leg]['gravity'][1])
        
    robot_gravity=[0,0,0]
    total_weight=0
    for leg in FL,FR,RL,RR:
        g=irobot['gravities'][leg]
        robot_gravity[X]+=g[0][X]*g[1]
        robot_gravity[Y]+=g[0][Y]*g[1]
        robot_gravity[Z]+=g[0][Z]*g[1]
        total_weight+=g[1]
    robot_gravity[X]/=total_weight
    robot_gravity[Y]/=total_weight
    robot_gravity[Z]/=total_weight
    irobot['gravity']=(robot_gravity,total_weight)

    irobot['projections']={}
    
    irobot['projections']['FL_FR_RL']=get_point_projection_on_plane(irobot['feets'][FL],irobot['feets'][FR],irobot['feets'][RL],irobot['gravity'][0])
    irobot['projections']['FR_RL_RR']=get_point_projection_on_plane(irobot['feets'][FR],irobot['feets'][RL],irobot['feets'][RR],irobot['gravity'][0])
    irobot['projections']['RL_RR_FL']=get_point_projection_on_plane(irobot['feets'][RL],irobot['feets'][RR],irobot['feets'][FL],irobot['gravity'][0])
    irobot['projections']['RR_FL_FR']=get_point_projection_on_plane(irobot['feets'][RR],irobot['feets'][FL],irobot['feets'][FR],irobot['gravity'][0])
    
def update_projections():
    canvas_width = int(projections[FL]['width'])
    canvas_height = int(projections[FL]['height'])
    shift_x=canvas_width/2
    shift_y=canvas_height/2

    for i in projections:
        projections[i].delete("all")
    
    for triangle in [ ((FL,FR,RL) , 'FL_FR_RL') , ((FR,RL,RR),'FR_RL_RR'), ((RL,RR,FL),'RL_RR_FL'),((RR,FL,FR),'RR_FL_FR') ]:
        points=[]
        for l in triangle[0]:
            px=ROBOT['feets'][l][X] * CANVAS_SCALE/2.0 + shift_x
            py=ROBOT['feets'][l][Y] * CANVAS_SCALE/2.0 + shift_y
            points.append((px,py))
        for i in range(3):
            projections[triangle[0][0]].create_line(points[i][X],points[i][Y],points[(i+1)%3][X],points[(i+1)%3][Y],fill="black",width=4)
        px=ROBOT['projections'][triangle[1]][0][X]* CANVAS_SCALE/2.0 + shift_x
        py=ROBOT['projections'][triangle[1]][0][Y]* CANVAS_SCALE/2.0 + shift_y
        if ROBOT['projections'][triangle[1]][1]:
            projections[triangle[0][0]].create_oval(px-2,py-2,px+2,py+2,fill="red")
        else:
            projections[triangle[0][0]].create_oval(px-2,py-2,px+2,py+2,fill="blue")
        projections[triangle[0][0]].create_text(px-2,py-10,text="%.2f"%ROBOT['projections'][triangle[1]][2])

    #for l in FL,FR,RL:
    #    c
    
    #top_view.create_oval(px-1,py-1,px+1,py+1,fill="#FFFFFF")

    
def update_top_view():
    global top_view
    top_view.delete("all")
    canvas_width = int(top_view['width'])
    canvas_height = int(top_view['height'])
    shift_x=canvas_width/2
    shift_y=canvas_height/2
    top_view.create_oval(shift_x,shift_y,shift_x+5,shift_y+5,fill="#476042")
    feets={}

    update_robot()    
    for leg in FL,FR,RL,RR:
        if (leg in ROBOT['feets'])==False:
            return
        px=ROBOT['feets'][leg][X] * CANVAS_SCALE + shift_x
        py=ROBOT['feets'][leg][Y] * CANVAS_SCALE + shift_y
        top_view.create_oval(px-5,py-5,px+5,py+5,fill="#FF0000")
        
        px=ROBOT['centers'][leg][X] * CANVAS_SCALE + shift_x
        py=ROBOT['centers'][leg][Y] * CANVAS_SCALE + shift_y
        top_view.create_oval(px-5,py-5,px+5,py+5,fill="#00FF00")

    # compute gravity center of the robot
    px=ROBOT['gravity'][0][X] * CANVAS_SCALE + shift_x
    py=ROBOT['gravity'][0][Y] * CANVAS_SCALE + shift_y
    top_view.create_oval(px-5,py-5,px+5,py+5,fill="#FF00FF")

    #h=get_point_projection_on_plane(ROBOT['feets'][FL],ROBOT['feets'][FR],ROBOT['feets'][RL],ROBOT['gravity'][0])
    #px=h[X] * CANVAS_SCALE + shift_x
    #py=h[Y] * CANVAS_SCALE + shift_y
    
    #top_view.create_oval(px-1,py-1,px+1,py+1,fill="#FFFFFF")

    update_projections()
    
def update_angle_kin(x,leg,robot=ROBOT):
    robot['LEGS'][leg]['angle']=robot['LEGS'][leg]['angle_orig'] + ((robot['LEGS'][leg]['v3sim'] - 0.45 ) / (0.650-0.45) - 0.5 ) * math.pi /3
        
def update_angle(x,leg,robot=ROBOT):
    robot['LEGS'][leg]['angle']=robot['LEGS'][leg]['angle_orig'] + ((robot['LEGS'][leg]['v3sim'] - 450.0 ) / (650.0-450.0) - 0.5 ) * math.pi /3
    #ANGLES[leg]=ANGLES_ORIG[leg] + ((LEGS[leg]['v3sim'] - 450.0 ) / (650.0-450.0) - 0.5 ) * math.pi /3
    update_transformation_matrices(robot['LEGS'])
    update_top_view()


def distance_point_to_line(p1,p2,p):
    return math.fabs((p2[Y] - p1[Y]) * p[X] - (p2[X] - p1[X])* p[Y] + p2[X]*p1[Y] - p2[Y]*p1[X]) / math.sqrt((p2[Y]-p1[Y])**2 + (p2[X] - p1[X])**2)
    
    
def get_point_projection_on_plane(p1,p2,p3,m):
    #print p1,p2,p3,m
    normale=normed(
        vectoriel3D(
            p2v(p1,p2),
            p2v(p1,p3)
        )
    )
    p=plus(
        m,
        prod(
            normale,
            -scalaire(p2v(p1,m),normale)
        )
    )
    # check if it is in triangle
    w1 = ( p1[X]*(p3[Y]-p1[Y]) + (p[Y] - p1[Y])*(p3[X]-p1[X])-p[X]*(p3[Y]-p1[Y])) \
         / \
         ( (p2[Y] - p1[Y])*(p3[X]-p1[X]) - (p2[X]-p1[X])*(p3[Y]-p1[Y]) )
    w2 = ( p[Y] - p1[Y] - w1*(p2[Y]-p1[Y]) ) / (p3[Y]-p1[Y])

    d1 = distance_point_to_line(p1,p2,p)
    d2 = distance_point_to_line(p1,p3,p)
    d3 = distance_point_to_line(p2,p3,p)
    
    return p , w1 >=0 and w2 >=0 and w1+w2<=1.0 , min(d1,d2,d3)


def update(x,leg):
    global lines,ao,ae,ef
    data=ROBOT['LEGS'][leg]
    v1=data['v1sim']
    v2=data['v2sim']

    #print "update:",v1,v2
    
    canvas=LEGS_CANVAS[leg]

        
    canvas.delete("all")
    msg=""
    Cmax=0
    Dmax=0
    Imax=0
    Hmax=0
    points=get_leg_points_V1_V2(v1/1000.0,
                                v2/1000.0)

    lpoints=dict(points)
    del lpoints['R']
    del lpoints['Q']
    data['gravity']=get_gravity_center(lpoints,weigths)
    
    
    tpoints=get_leg_points_V1_V2(450.0/1000.0,
                                 450.0/1000.0)
    forces={}
    stress={}
    compute_forces_and_stress(tpoints,weigths,forces,stress)
    msg+="450 450: \n"
    msg+="\tCD en C: %.2fN\n"%stress['C']
    msg+="\tCD en D: %.2fN\n"%stress['D']
    msg+="\tHI en I: %.2fN\n"%stress['I']
    msg+="\tHI en H: %.2fN\n"%stress['H']
    Cmax=max(Cmax,stress['C'])
    Dmax=max(Dmax,stress['D'])
    Imax=max(Imax,stress['I'])
    Hmax=max(Hmax,stress['H'])
    
    
    points['J1']=tpoints['J']
    tpoints=get_leg_points_V1_V2(650.0/1000.0,
                           450.0/1000.0)
    forces={}
    stress={}
    compute_forces_and_stress(tpoints,weigths,forces,stress)
    msg+="650 450: \n"
    msg+="\tCD en C: %.2fN\n"%stress['C']
    msg+="\tCD en D: %.2fN\n"%stress['D']
    msg+="\tHI en I: %.2fN\n"%stress['I']
    msg+="\tHI en H: %.2fN\n"%stress['H']
    Cmax=max(Cmax,stress['C'])
    Dmax=max(Dmax,stress['D'])
    Imax=max(Imax,stress['I'])
    Hmax=max(Hmax,stress['H'])
    
    points['J2']=tpoints['J']
    tpoints=get_leg_points_V1_V2(450.0/1000.0,
                           650.0/1000.0)
    forces={}
    stress={}
    compute_forces_and_stress(tpoints,weigths,forces,stress)
    msg+="450 650: \n"
    msg+="\tCD en C: %.2fN\n"%stress['C']
    msg+="\tCD en D: %.2fN\n"%stress['D']
    msg+="\tHI en I: %.2fN\n"%stress['I']
    msg+="\tHI en H: %.2fN\n"%stress['H']
    Cmax=max(Cmax,stress['C'])
    Dmax=max(Dmax,stress['D'])
    Imax=max(Imax,stress['I'])
    Hmax=max(Hmax,stress['H'])
    
    points['J3']=tpoints['J']
    tpoints=get_leg_points_V1_V2(650.0/1000.0,
                           650.0/1000.0)
    forces={}
    stress={}
    compute_forces_and_stress(tpoints,weigths,forces,stress)
    msg+="650 650: \n"
    msg+="\tCD en C: %.2fN\n"%stress['C']
    msg+="\tCD en D: %.2fN\n"%stress['D']
    msg+="\tHI en I: %.2fN\n"%stress['I']
    msg+="\tHI en H: %.2fN\n"%stress['H']
    Cmax=max(Cmax,stress['C'])
    Dmax=max(Dmax,stress['D'])
    Imax=max(Imax,stress['I'])
    Hmax=max(Hmax,stress['H'])
    points['J4']=tpoints['J']

    forces={}
    stress={}
    compute_forces_and_stress(points,weigths,forces,stress)
    

    msg+="CD en C: %.2fN (%.2f)\n"%(stress['C'],Cmax)
    msg+="CD en D: %.2fN (%.2f)\n"%(stress['D'],Dmax)
    msg+="HI en I: %.2fN (%.2f)\n"%(stress['I'],Imax)
    msg+="HI en H: %.2fN (%.2f)\n"%(stress['H'],Hmax)

    data['points']=points
    data['forces']=forces
    data['stress']=stress
    
    draw_points(canvas,points,lines,forces,msg)
    update_top_view()


def update_all(x):
    update(x,FL)
    update(x,FR)
    update(x,RL)
    update(x,RR)

    
from scipy.optimize import minimize


def inverse_kinetic_fun(v,dx,dy):
    for k in v:
        if k<0.449 or k>0.651:
            print("!!!!! out of bounds",k)
            return None
    p=get_leg_points_V1_V2(v[0],v[1])
    return norm(p2v(p['J'],(dx,dy)))

def inverse_kinetic(x,y):    
    res=minimize(inverse_kinetic_fun,(0.55,0.55),args=(x,y),\
                 #method='Nelder-Mead',\
                 bounds=((0.45,0.65),(0.45,0.65)),
                 tol=1e-6)
    p=get_leg_points_V1_V2(res.x[0],res.x[1])
    return (res.x[0],res.x[1],p,norm(p2v(p['J'],(x,y))))


import copy

def kinetic_for_gravity_center_fix_feet(center,triangle,legs):
    # move center without moving feets from legs
    robot=copy.deepcopy(ROBOT)
    nv={}    
    for l in legs:
        nv[l]={}
        feet=robot['feets'][l]
        nfeet=list(feet)
        for v in range(len(center)):
            nfeet[v]-=center[v]
#        print "****"
#        print "center ",center
#        print "feet ",feet,"nfeet ",nfeet,"origin ",robot['LEGS'][l]['origin']
#        print "angle ",robot['LEGS'][l]['angle']
        # compute angle delta:
        #t1=math.atan2(robot['LEGS'][l]['origin'][Y] - feet[Y],robot['LEGS'][l]['origin'][X] - feet[X])
        t2=math.atan2(nfeet[Y]-robot['LEGS'][l]['origin'][Y] , nfeet[X] -  robot['LEGS'][l]['origin'][X])
        if t2<0:
            t2+=math.pi*2.0
#        print "t2:",t2        
        delta=t2-robot['LEGS'][l]['angle_orig']
        if math.fabs(delta) > math.pi/3:
            # is not possible for v3
            delta = math.copysign(math.pi/3,delta)
        robot['LEGS'][l]['angle'] = t2
#        print "v3sim was ",robot['LEGS'][l]['v3sim'] 
        robot['LEGS'][l]['v3sim'] = ((delta*3/math.pi + 0.5) * (0.65-0.45) + 0.45)*1000.0
#        print "v3sim is ",delta,robot['LEGS'][l]['v3sim']        

        update_transformation_matrices(robot['LEGS'])
        
#        print "point J in leg base was ",robot['LEGS'][l]['points']['J']
#        print "J in robot base was ",numpy.matmul(robot['LEGS'][l]['matrix'],robot['LEGS'][l]['points']['J']+(0,1)).tolist()
#        print "J in robot base was ",robot['feets'][l]
#        print "point J in leg base using reverse matrix",numpy.matmul(robot['LEGS'][l]['imatrix'],robot['feets'][l]+[1]).tolist()[:3]
        
#        print robot['LEGS'][l]['imatrix']
        lfeet=numpy.matmul(robot['LEGS'][l]['imatrix'],nfeet+[1]).tolist()
#        print "point J was ",robot['LEGS'][l]['points']['J']
#        print "lfeet is ",lfeet[:3]
        rev=inverse_kinetic(lfeet[X],lfeet[Y])
        malus=0.0
        if rev[3]>0.01:
            print("OVER MOVE!")
            malus=10.0
        robot['LEGS'][l]['v1sim'] = rev[0]*1000.0
        robot['LEGS'][l]['v2sim'] = rev[1]*1000.0
        nv[l]['v1sim']=robot['LEGS'][l]['v1sim']
        nv[l]['v2sim']=robot['LEGS'][l]['v2sim']
        nv[l]['v3sim']=robot['LEGS'][l]['v3sim']
        update_angle_kin(None,l,robot)
    update_transformation_matrices(robot['LEGS'])
    update_robot(robot)    
    if robot['projections'][triangle][1]:
        #print v, -robot['projections'][triangle][2]
        return -robot['projections'][triangle][2]+malus,nv
    return robot['projections'][triangle][2]+malus,nv


def kinetic_for_gravity_center_fix_feet_fun(center,triangle,legs):
    res=kinetic_for_gravity_center_fix_feet(center,triangle,legs)
    return res[0]
    

# following is wrong and must be removed!
def kinetic_for_gravity_center(v,triangle,direction,legs):
    #print v,triangle,direction,legs
    for k in v:
        if k<0.449 or k>0.651:
            return 1000.0
    robot=copy.deepcopy(ROBOT)
    i=0
    for l in legs:
        robot['LEGS'][l]['v1sim']=v[i]
        i+=1
        robot['LEGS'][l]['v2sim']=v[i]
        i+=1
        robot['LEGS'][l]['v3sim']=v[i]
        i+=1
    for l in legs:
        #print robot['LEGS'][l]['v1sim']
        #print robot['LEGS'][l]['v2sim']
        robot['LEGS'][l]['points']=get_leg_points_V1_V2(robot['LEGS'][l]['v1sim'],\
                                                        robot['LEGS'][l]['v2sim'])
        robot['LEGS'][l]['gravity']=get_gravity_center(robot['LEGS'][l]['points'],weigths)
        update_angle_kin(None,l,robot)
    update_transformation_matrices(robot['LEGS'])
    update_robot(robot)
    #print "direction:",direction,"gravity projection:",robot['projections'][triangle]

    #n=norm(p2v(robot['feets'][direction],robot['projections'][triangle][0]))
    #if robot['projections'][triangle][1] and robot['projections'][triangle][2]>0.01:
    #    return n
    #print "distance:",n
    #return n*2
    #print v, triangle, robot['projections'][triangle]
    if robot['projections'][triangle][1]:
        #print v, -robot['projections'][triangle][2]
        return -robot['projections'][triangle][2]
    return robot['projections'][triangle][2]

def onclick(event,leg):
    v1 = LEGS_SLIDER[leg]['v1']
    v2 = LEGS_SLIDER[leg]['v2']
    canvas = LEGS_CANVAS[leg]
    cx=float(event.x)
    cy=float(event.y)
    x=(cx-float(canvas['width'])/2.0) / CANVAS_SCALE
    y=-(cy-float(canvas['height'])/2.0) / CANVAS_SCALE

    res=inverse_kinetic(x,y)
    v1.set(int(res[0]*1000))
    v2.set(int(res[1]*1000))

    # balance body on other three legs:

    if leg==FL:
        triangle='FR_RL_RR'
        direction=FR
        legs=(FR,RL,RR)
    elif leg==FR:
        triangle='RL_RR_FL'
        direction=FL
        legs=(RL,RR,FL)
    elif leg==RL:
        triangle='RR_FL_FR'
        direction=FR
        legs=(RR,FL,FR)
    else:
        triangle='FL_FR_RL'
        direction=FL
        legs=(FL,FR,RL)
    initial_values=[0.0,0.0,0]
    bounds=((-0.1,0.1),(-0.1,0.1),(-0.1,0.1))
    res=minimize(kinetic_for_gravity_center_fix_feet_fun,initial_values,args=(triangle,legs),\
                 #method='Nelder-Mead',\
                 bounds=bounds,
                 tol=1e-6)
    nv=kinetic_for_gravity_center_fix_feet(res.x,triangle,legs)
#    nv=kinetic_for_gravity_center_fix_feet([0.0,0,0],triangle,legs)
    print(res)
    print(nv)
    for l in legs:
        LEGS_SLIDER[l]['v1'].set(int(round(nv[1][l]['v1sim'])))
        LEGS_SLIDER[l]['v2'].set(int(round(nv[1][l]['v2sim'])))
        LEGS_SLIDER[l]['v3'].set(int(round(nv[1][l]['v3sim'])))
#    initial_values=[]
#    bounds=[]
#    for l in legs:
#        initial_values.append(float(LEGS_SLIDER[l]['v1'].get())/1000.0)
#        bounds.append((0.45,0.65))
#        initial_values.append(float(LEGS_SLIDER[l]['v2'].get())/1000.0)
#        bounds.append((0.45,0.65))
#        initial_values.append(float(LEGS_SLIDER[l]['v3'].get())/1000.0)
#        bounds.append((0.45,0.65))
#    res=minimize(kinetic_for_gravity_center,initial_values,args=(triangle,direction,legs),\
#                 #method='Nelder-Mead',\
#                 bounds=bounds,
#                 tol=1e-9)
#    i=0
#    for l in legs:
#        LEGS_SLIDER[l]['v1'].set(int(res.x[i]*1000.0))
#        i+=1
#        LEGS_SLIDER[l]['v2'].set(int(res.x[i]*1000.0))
#        i+=1
#       LEGS_SLIDER[l]['v3'].set(int(res.x[i]*1000.0))
#        i+=1
#    print res

def move_gravity_center(event):
    global top_view
    cx=float(event.x)
    cy=float(event.y)
    x=(cx-float(top_view['width'])/2.0) / CANVAS_SCALE
    y=-(cy-float(top_view['height'])/2.0) / CANVAS_SCALE
    print("move robot center to:",x,y)
    res=kinetic_for_gravity_center_fix_feet((x,y),'FL_FR_RL',(FL,FR,RL,RR))
    for l in (FL,FR,RL,RR):
        LEGS_SLIDER[l]['v1'].set(int(res[1][l]['v1sim']))
        LEGS_SLIDER[l]['v2'].set(int(res[1][l]['v2sim']))
        LEGS_SLIDER[l]['v3'].set(int(res[1][l]['v3sim']))






        
master = Tk()

canvas=Frame(master)

flcanvas = Canvas(canvas, 
                  width=400,
                  height=400)
CANVAS_SCALE=150
flcanvas.bind("<Button-1>",lambda e: onclick(e,FL))
flcanvas.pack(side=LEFT)

LEGS_CANVAS[FL]=flcanvas

frcanvas = Canvas(canvas, 
                width=400,
                height=400)
frcanvas.bind("<Button-1>",lambda e: onclick(e,FR))
frcanvas.pack(side=LEFT)

LEGS_CANVAS[FR]=frcanvas

rlcanvas = Canvas(canvas, 
                width=400,
                height=400)
rlcanvas.bind("<Button-1>",lambda e: onclick(e,RL))
rlcanvas.pack(side=LEFT)

LEGS_CANVAS[RL]=rlcanvas

rrcanvas = Canvas(canvas, 
                width=400,
                height=400)
rrcanvas.bind("<Button-1>",lambda e: onclick(e,RR))
rrcanvas.pack(side=LEFT)

LEGS_CANVAS[RR]=rrcanvas

canvas.grid(row=0)

robot=Frame(master)

top_view=Canvas(robot,
                width=400,
                height=400)
top_view.bind("<Button-1>",move_gravity_center)
top_view.pack(side=LEFT)

projections_frame=Frame(robot)

projections={}
projections[FL]=Canvas(projections_frame,
                       width=200,
                       height=200)
projections[FL].grid(row=0,column=0)
projections[FR]=Canvas(projections_frame,
                       width=200,
                       height=200)
projections[FR].grid(row=0,column=1)
projections[RL]=Canvas(projections_frame,
                       width=200,
                       height=200)
projections[RL].grid(row=1,column=0)
projections[RR]=Canvas(projections_frame,
                       width=200,
                       height=200)
projections[RR].grid(row=1,column=1)

projections_frame.pack(side=LEFT)

robot.grid(row=1)







verins=Frame(master)

Label(verins,text="FL V1:").pack(side=LEFT)
flv1 = Scale(verins, from_=450, to=650, orient=HORIZONTAL, command=lambda x: update(x,FL))
flv1.set(550)
flv1.pack(side=LEFT)

Label(verins,text="V2:").pack(side=LEFT)
flv2 = Scale(verins, from_=450, to=650, orient=HORIZONTAL, command=lambda x: update(x,FL))
flv2.set(550)
flv2.pack(side=LEFT)

Label(verins,text="V3:").pack(side=LEFT)
flv3 = Scale(verins, from_=450, to=650, orient=HORIZONTAL, command=lambda x: update_angle(x,FL))
flv3.set(550)
flv3.pack(side=LEFT)


LEGS_SLIDER[FL]['v1']=flv1
LEGS_SLIDER[FL]['v2']=flv2
LEGS_SLIDER[FL]['v3']=flv3

Label(verins,text="FR V1:").pack(side=LEFT)
frv1 = Scale(verins, from_=450, to=650, orient=HORIZONTAL, command=lambda x: update(x,FR) )
frv1.set(550)
frv1.pack(side=LEFT)

Label(verins,text="V2:").pack(side=LEFT)
frv2 = Scale(verins, from_=450, to=650, orient=HORIZONTAL, command=lambda x: update(x,FR) )
frv2.set(550)
frv2.pack(side=LEFT)

Label(verins,text="V3:").pack(side=LEFT)
frv3 = Scale(verins, from_=450, to=650, orient=HORIZONTAL, command=lambda x: update_angle(x,FR) )
frv3.set(550)
frv3.pack(side=LEFT)

LEGS_SLIDER[FR]['v1']=frv1
LEGS_SLIDER[FR]['v2']=frv2
LEGS_SLIDER[FR]['v3']=frv3

Label(verins,text="RL V1:").pack(side=LEFT)
rlv1 = Scale(verins, from_=450, to=650, orient=HORIZONTAL, command=lambda x: update(x,RL) )
rlv1.set(550)
rlv1.pack(side=LEFT)

Label(verins,text="V2:").pack(side=LEFT)
rlv2 = Scale(verins, from_=450, to=650, orient=HORIZONTAL, command=lambda x: update(x,RL) )
rlv2.set(550)
rlv2.pack(side=LEFT)

Label(verins,text="V3:").pack(side=LEFT)
rlv3 = Scale(verins, from_=450, to=650, orient=HORIZONTAL, command=lambda x: update_angle(x,RL) )
rlv3.set(550)
rlv3.pack(side=LEFT)

LEGS_SLIDER[RL]['v1']=rlv1
LEGS_SLIDER[RL]['v2']=rlv2
LEGS_SLIDER[RL]['v3']=rlv3

Label(verins,text="RR V1:").pack(side=LEFT)
rrv1 = Scale(verins, from_=450, to=650, orient=HORIZONTAL, command=lambda x: update(x,RR) )
rrv1.set(550)
rrv1.pack(side=LEFT)

Label(verins,text="V2:").pack(side=LEFT)
rrv2 = Scale(verins, from_=450, to=650, orient=HORIZONTAL, command=lambda x: update(x,RR) )
rrv2.set(550)
rrv2.pack(side=LEFT)

Label(verins,text="V3:").pack(side=LEFT)
rrv3 = Scale(verins, from_=450, to=650, orient=HORIZONTAL, command=lambda x: update_angle(x,RR) )
rrv3.set(550)
rrv3.pack(side=LEFT)

LEGS_SLIDER[RR]['v1']=rrv1
LEGS_SLIDER[RR]['v2']=rrv2
LEGS_SLIDER[RR]['v3']=rrv3

verins.grid(row=2,sticky=W)

geometry=Frame(master)

Label(geometry,text="AO:").pack(side=LEFT)
ao = Scale(geometry, from_=100, to=300, orient=HORIZONTAL, command=update_all)
ao.set(150)
ao.pack(side=LEFT)

l=Label(geometry,text="BC:").pack(side=LEFT)
bc = Scale(geometry, from_=150, to=400, orient=HORIZONTAL, command=update_all)
bc.set(210)
bc.pack(side=LEFT)

Label(geometry,text="BF:").pack(side=LEFT)
bf = Scale(geometry, from_=500, to=700, orient=HORIZONTAL, command=update_all)
bf.set(600)
bf.pack(side=LEFT)

Label(geometry,text="AE:").pack(side=LEFT)
ae = Scale(geometry, from_=400, to=700, orient=HORIZONTAL, command=update_all)
ae.set(500)
ae.pack(side=LEFT)

Label(geometry,text="DE:").pack(side=LEFT)
de = Scale(geometry, from_=100, to=400, orient=HORIZONTAL, command=update_all)
de.set(200)
de.pack(side=LEFT)
    
Label(geometry,text="EF:").pack(side=LEFT)
ef = Scale(geometry, from_=400, to=600, orient=HORIZONTAL, command=update_all)
ef.set(450)
ef.pack(side=LEFT)
    
l=Label(geometry,text="FG:").pack(side=LEFT)
fg = Scale(geometry, from_=100, to=300, orient=HORIZONTAL, command=update_all)
fg.set(150)
fg.pack(side=LEFT)

Label(geometry,text="FH:").pack(side=LEFT)
fh = Scale(geometry, from_=200, to=500, orient=HORIZONTAL, command=update_all)
fh.set(200)
fh.pack(side=LEFT)

Label(geometry,text="GI:").pack(side=LEFT)
gi = Scale(geometry, from_=300, to=700, orient=HORIZONTAL, command=update_all)
gi.set(450)
gi.pack(side=LEFT)

Label(geometry,text="GJ:").pack(side=LEFT)
gj = Scale(geometry, from_=500, to=1500, orient=HORIZONTAL, command=update_all)
gj.set(1000)
gj.pack(side=LEFT)

geometry.grid(row=3,sticky=W)

def config1():
    ao.set(150) 
    bc.set(210)
    bf.set(600)
    ae.set(500)
    de.set(200)
    ef.set(450)
    fg.set(150)
    fh.set(200)
    gi.set(450)
    gj.set(1000)
    
def config2():
    ao.set(150) # idem
    bc.set(300)
    bf.set(600) # idem
    ae.set(500) # idem
    de.set(100)
    ef.set(450) # idem
    fg.set(300) 
    fh.set(300)
    gi.set(500)
    gj.set(1000)


walk={'enabled':False,'current_leg':FL,'current_phase':'start'}
TRIANGLES={FL:'FR_RL_RR',FR:'RL_RR_FL',RL:'RR_FL_FR',RR:'FL_FR_RL'}

def update_linears_actuators():
    global walk
    mod=False
    for l in FL,FR,RL,RR:
        for v in 'v1','v2','v3':
            goal=int(LEGS_SLIDER[l][v].get())
            delta=goal-LEGS[l][v+'sim']
            if math.fabs(delta)>0:
                if math.fabs(delta)<(UPDATE_FREQ/1000.0*TIME_FACTOR*SPEED[v]):
                    LEGS[l][v+'sim']=int(goal)
                else:
                    LEGS[l][v+'sim']=LEGS[l][v+'sim']+UPDATE_FREQ/1000.0*TIME_FACTOR*SPEED[v] * delta/math.fabs(delta)
                #print "timer:",l,v,goal, int(LEGS[l][v].get()), LEGS[l][v+'sim'],delta
                mod=True
                #walk['enabled']=True
        if mod:
            update(None,l)
            update_angle(None,l)
    if walk['enabled']==True:
        print(walk)
        #check if sim value are ok with current values:
        if mod==False: # all programmed movements are done
            # prepare the next movement
            if walk['current_phase']=='start':            
                initial_values=[0.0,0.0,0]
                bounds=((-0.2,0.2),(-0.2,0.2),(-0.2,0.2))
                res=minimize(kinetic_for_gravity_center_fix_feet_fun,initial_values,args=(TRIANGLES[walk['current_leg']],(FL,FR,RL,RR)),\
                             #method='Nelder-Mead',\
                             bounds=bounds,
                             tol=1e-6)
                print("try to move gravity center on the triangle opposite to the moving leg:")
                print("res is:",res)
                nv=kinetic_for_gravity_center_fix_feet(res.x,TRIANGLES[walk['current_leg']],(FL,FR,RL,RR))
                print("nv is:",nv)
                for l in FL,FR,RL,RR:
                    print("setting actuator values for leg ",l,nv[1][l])
                    for v in 'v1','v2','v3':
                        LEGS_SLIDER[l][v].set(round(nv[1][l][v+'sim']))
                    print("setting actuator values for leg ",[  LEGS_SLIDER[l][v].get() for v in LEGS_SLIDER[l]])
                walk['current_phase']='start_gravity_wait'
                walk['enabled']=False
            elif walk['current_phase']=='start_gravity_wait':
                print("raise leg 5 cm over the ground")
                feet=ROBOT['feets'][walk['current_leg']]
                nfeet=list(feet)
                nfeet[Z]+=0.05
                lfeet=numpy.matmul(ROBOT['LEGS'][walk['current_leg']]['imatrix'],nfeet+[1]).tolist()
                rev=inverse_kinetic(lfeet[X],lfeet[Y])
                LEGS_SLIDER[walk['current_leg']]['v1'].set(rev[0]*1000.0)
                LEGS_SLIDER[walk['current_leg']]['v2'].set(rev[1]*1000.0)
                walk['current_phase']='moving_up'
                walk['enabled']=False
                print("DEBUG:",walk)
            elif walk['current_phase']=='moving_up':
                # need to compute next leg position according to direction
                walk['current_phase']='moving'
            elif walk['current_phase']=='moving':
                np=get_point_projection_on_plane(ROBOT['feets'][FR],ROBOT['feets'][RL],ROBOT['feets'][RR],ROBOT['feets'][walk['current_leg']])
                print(np)
                lfeet=numpy.matmul(ROBOT['LEGS'][walk['current_leg']]['imatrix'],list(np[0])+[1]).tolist()
                rev=inverse_kinetic(lfeet[X],lfeet[Y])
                LEGS_SLIDER[walk['current_leg']]['v1'].set(rev[0]*1000.0)
                LEGS_SLIDER[walk['current_leg']]['v2'].set(rev[1]*1000.0)
                walk['current_phase']='moving_down'
            elif walk['current_phase']=='moving_down':
                if walk['current_leg']==FL:
                    walk['current_leg']=RR
                elif walk['current_leg']==RR:
                    walk['current_leg']=FR
                elif walk['current_leg']==FR:
                    walk['current_leg']=RL
                elif walk['current_leg']==RL:
                    walk['current_leg']=FL
                walk['current_phase']='start'

                 
            print("DEBUG2:",walk)
                
                
                
            
    master.after(int(UPDATE_FREQ),update_linears_actuators)
            
    
    

configs=Frame(master)
    
conf1 = Button(configs, text="Config 1",command=config1)
conf1.pack(side=LEFT)

conf2 = Button(configs, text="Config 2",command=config2)
conf2.pack(side=LEFT)

def switch_walk():
    global walk,walkButton_variable
    print("switch")
    if walk['enabled']:
        walk['enabled']=False
        walkButton_variable.set("off")
    else:
        walk['enabled']=True



walkButton_variable = StringVar(value="off")
walkButton = Radiobutton(configs, text="walk",command=switch_walk,variable=walkButton_variable,indicatoron=True)
walkButton.pack(side=LEFT)

configs.grid(row=4,sticky=W)

config2()

#draw_points(canvas,points,lines)


update_transformation_matrices(LEGS)

for angle in range(-30,30,5):
    LEGS[FL]['angle'] = LEGS[FL]['angle_orig'] + math.radians(angle)    
    update_transformation_matrices(LEGS)
    for v1 in range(450,650,1):    
        for v2 in range(450,650,1):
            p=get_leg_points_V1_V2(v1/1000.0,v2/1000.0)        
            feet=p['J']
            feet=(feet[0],feet[1],0,1)
            rfeet=numpy.matmul(LEGS[FL]['matrix'],feet).tolist()
            print(v1,v2,p['J'][X],p['J'][Y],round(rfeet[X]*1000),round(rfeet[Y]*1000),round(rfeet[Z]*1000))
sys.exit(1)
