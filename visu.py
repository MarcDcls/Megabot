from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import OpenGL.GL as ogl
from threading import Thread

import time
import random

from kinetic import *

colors = np.array([
    [1, 1, 1, 1.0],
    [1, 0, 0, 1.0],
    [0, 1, 0, 1.0],
    [0, 0, 1, 1.0]
])

class Visu(Thread):
    def __init__(self,controlers):
        Thread.__init__(self)
        print("visu 0")
        self.controlers=controlers
        self.start_time=time.time()
        self.last_displayed=10.0
        self.paused=False
    def pause(self):
        self.paused=not self.paused
    def setTimeDisplay(self,r):
        self.last_displayed=r;
    def getTimeDisplay(self):
        return self.last_displayed
    def _update(self):
        cont=0
        if self.paused:
            return
        for p in self.plots:
            actu=0
            for v in p:
                v['times'].append(time.time()-self.start_time)
                v['position data'].append(self.controlers[cont].la[actu]['position'])
                v['position plot'].setData(v['times'],v['position data'])
                v['position'].setXRange(v['times'][-1]-self.last_displayed,v['times'][-1])
                
                v['target data'].append(self.controlers[cont].la[actu]['target'])
                v['target plot'].setData(v['times'],v['target data'])
                v['target'].setXRange(v['times'][-1]-self.last_displayed,v['times'][-1])
                                
                v['last data'].append(self.controlers[cont].la[actu]['last_order'])
                v['last plot'].setData(v['times'],v['last data'])
                v['last'].setXRange(v['times'][-1]-self.last_displayed,v['times'][-1])
                actu+=1
            cont+=1
        for l in self.legs:
            try:
                self.wingl.removeItem(l)
            except:
                pass
        self.legs=[]
        minz=0
        for l in FL,FR,RR,RL:
            v1=(self.controlers[l].la[1]['position']+450.0)/1000.0
            v2=(self.controlers[l].la[2]['position']+450.0)/1000.0
            v3=(self.controlers[l].la[0]['position']+450.0)/1000.0
            for v in [v1,v2,v3]:
                if v<0.43 or v>0.66:
                    print("VISU ERROR:",v)
                    return
            points=robot_ref_leg_points(LEGS,l,v1,v2,v3)
            minz=min(minz,points['J'][Z])
            self.legs_buffer.append(points['J'])
            p=[]
            for t in ('C','A','E','G','J'):
                p.append(points[t])
            item=gl.GLLinePlotItem(pos=np.array(p),color=(1,1,1,1),width=5)
            self.legs.append(item)
        for l in FL,FR,RR,RL:
            v1=(self.controlers[l].la[1]['target']+450.0)/1000.0
            v2=(self.controlers[l].la[2]['target']+450.0)/1000.0
            v3=(self.controlers[l].la[0]['target']+450.0)/1000.0
            for v in [v1,v2,v3]:
                if v<0.43 or v>0.66:
                    print("VISU ERROR:",v)
                    return
            points=robot_ref_leg_points(LEGS,l,v1,v2,v3)
            p=[]
            for t in ('C','A','E','G','J'):
                p.append(points[t])
            item=gl.GLLinePlotItem(pos=np.array(p),color=(1,0,0,1),width=5)
            self.legs.append(item)
        p=[]
        for l in FL,FR,RR,RL,FL:
            p.append(LEGS[l]['origin'])
        
        self.legs.append(gl.GLLinePlotItem(pos=np.array(p)))
        
        self.legs_buffer=self.legs_buffer[-320:]
        sp=gl.GLScatterPlotItem(pos=np.array(self.legs_buffer),size=np.full((len(self.legs_buffer)),0.05), color=(0,1,0,1), pxMode=False)
        self.legs.append(sp)

        for item in self.legs:
            item.translate(0,0,-minz)
            self.wingl.addItem(item)
        #self.wingl.pan(0,0,self.glpanz-minz)
        #self.glpanz=minz

    def quit(self):
        QtGui.QApplication.exit()
        self.join()
    
    def run(self):
        self.app = QtGui.QApplication([])
        self.win = pg.GraphicsWindow(title="Controler outputs",border=True)        
        self.win.resize(3000,1800)

        shortcut = pg.QtGui.QShortcut(pg.QtGui.QKeySequence(' '), self.win)
        shortcut.activated.connect(self.pause)
        
#        gz = gl.GLGridItem()
#        gz.setSpacing(x=0.05,y=0.05,z=0.1)
#        self.wingl.addItem(gz)

        
        
        self.legs_buffer=[]
        #self.glpanz=0

        self.legs=[]
                  
        print("visu 1")
        
        pg.setConfigOptions(antialias=True)

        self.plots=[None,None,None,None]
        self.layout=[]
        row=[0,1,2,3]
        col=[0,0,0,0]
        i=0
        
        print("visu 2")
        
        for c in self.controlers:
            self.plots[c.id]=[]
            self.layout.append(self.win.addLayout(row=row[i],col=col[i]))
            i+=1

            vid=0
            for v in c.la:
                p1=self.layout[-1].addPlot(title="%d: %d position"%(c.id,vid))
                p1.setYRange(0,200)
                p1a=pg.PlotCurveItem(pen=(0,255,0))
                p1.addItem(p1a)
                p1b=pg.PlotCurveItem(pen=(255,0,0))
                p1.addItem(p1b)
                #p2=self.layout[-1].addPlot(title="%d: %d target"%(c.id,vid))
                #p2.setYRange(0,200)
                p3=self.layout[-1].addPlot(title="%d: %d last order"%(c.id,vid))
                p3.setYRange(-255,255)
                self.plots[c.id].append({'position':p1,
                                         'position plot':p1a,
                                         'position data':[],
                                         'target':p1,
                                         'target plot':p1b,
                                         'target data':[],
                                         'last':p3,
                                         'last plot':p3.plot(),
                                         'last data':[],
                                         'times':[]
                })
                vid+=1
            self.win.nextRow()

        self.wingl = gl.GLViewWidget()
        self.wingl.opts['distance'] = 5
        self.wingl.resize(1000,1000)
        self.wingl.show()
        self.wingl.setWindowTitle('3D robot')
        
        
        #ogl.glLight(ogl.GL_LIGHT0, ogl.GL_POSITION,  (5, 5, 5, 1))
        #ogl.glLightfv(ogl.GL_LIGHT0, ogl.GL_AMBIENT, (0, 0, 0, 1))
        #ogl.glLightfv(ogl.GL_LIGHT0, ogl.GL_DIFFUSE, (1, 1, 1, 1))
        #ogl.glEnable(ogl.GL_DEPTH_TEST) 
        x = np.linspace(-8, 8, 2)
        y = np.linspace(-8, 8, 2)
        z = np.array([ [0,0] , [0,0] ])
        p2 = gl.GLSurfacePlotItem(x=x, y=y, z=z ,shader='shaded',color=(0.5,0.5,0.5,0.5) )#glOptions='translucent')
        #s=gl.GLSurfacePlotItem(x=np.array([-1,-1,1,1]),y=np.array([-1,1,1,-1]),z=np.array([[0],[0],[0],[0]]),colors=(1,1,1,0.5),glOptions='translucent')
        self.wingl.addItem(p2)

        self.timer=QtCore.QTimer()
        self.timer.timeout.connect(self._update)
        self.timer.start(50)
        self.app.exec_()
