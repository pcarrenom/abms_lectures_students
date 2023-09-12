#!/usr/bin/env python
# Code adapted from https://github.com/MengGuo/RVO_Py_MAS/tree/master
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.animation as mpl_animation
import matplotlib as mpl
import numpy as np

from pedestrians_rvo.rvo import RVO_update, compute_V_des
# from rvo import RVO_update, compute_V_des


class rvo_visualizer():

    def __init__(self, positions, velocities, goals, vel_max, model, t_delta):
        self.positions = positions
        self.velocities = velocities
        self.goals = goals
        self.vel_max = vel_max
        self.model = model
        self.t_delta = t_delta

        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(1,1,1)
        self.humans = []
        self.humans_velocities = []

        self.human_collection = PatchCollection([])
        self.human_collection.set(animated=True, clip_on=True, alpha=0.6, cmap='tab20')
        self.vel_collection = PatchCollection([])
        self.vel_collection.set(animated=True, clip_on=True)


    def plot_obstacles(self):
        for hole in self.model['obstacles']:
            srec = matplotlib.patches.Rectangle(
                    (hole[0]-hole[2], hole[1]-hole[2]),
                    2*hole[2], 2*hole[2],
                    facecolor= 'red',
                    fill = True,
                    alpha=1)
            self.ax.add_patch(srec)

    def plot_goals(self):
        cmap = mpl.colormaps['tab20']
        # cmap = get_cmap(len(self.goals))
        for i in range(len(self.goals)):
            self.ax.plot([self.goals[i][0]], [self.goals[i][1]], '*', color=cmap(i), markersize =15,linewidth=3.0)


    def plot_humans(self):
        radius = self.model['radius']
        cmap = mpl.colormaps['tab20']
        # cmap = get_cmap(len(self.positions))
        humans = []
        humans_velocities = []

        if self.humans:
            for i, human in (enumerate(self.humans)):
                human.center = (self.positions[i][0],self.positions[i][1])
                human.radius = radius
                human.facecolor = cmap(i)

            for i, human_velocity in (enumerate(self.humans_velocities)):
                human_velocity.set_data(x=self.positions[i][0], y=self.positions[i][1], dx=self.velocities[i][0], dy=self.velocities[i][1])

        else:
            for i in range(0,len(self.positions)):
                #-------plot car
                robot = matplotlib.patches.Circle(
                    (self.positions[i][0],self.positions[i][1]),
                    radius = radius,
                    edgecolor='black',
                    facecolor=cmap(i),
                    linewidth=1.0,
                    ls='solid',
                    zorder=2)
                humans.append(robot)
                # ----------plot velocity
                r_vel = matplotlib.patches.FancyArrow(self.positions[i][0], self.positions[i][1], self.velocities[i][0], self.velocities[i][1], head_width=0.1, head_length=0.2, width=0.1)
                humans_velocities.append(r_vel)
            
            self.humans = humans
            self.humans_velocities = humans_velocities
        
            r_vel = matplotlib.patches.Arrow(self.positions[i][0], self.positions[i][1], self.velocities[i][0], self.velocities[i][1], width=0.2)
            
        self.human_collection.set_paths(self.humans)
        self.human_collection.set_facecolor([cmap(i) for i in range(0, len(self.humans))])
        self.vel_collection.set_paths(self.humans_velocities)
        self.vel_collection.set_facecolor([cmap(i) for i in range(0, len(self.humans))])

    def animation_update(self, i):
        # compute desired vel to goal
        V_des = compute_V_des(self.positions, self.goals, self.vel_max)
        # compute the optimal vel to avoid collision
        self.velocities = RVO_update(self.positions, V_des, self.velocities, self.model)
        # update position
        for j in range(len(self.velocities)):
            self.positions[j][0] += self.velocities[j][0]*self.t_delta
            self.positions[j][1] += self.velocities[j][1]*self.t_delta

        self.plot_humans()                        
        return (self.human_collection, self.vel_collection)

    def animation_init(self):
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-1.0, 6.0)
        self.ax.set_ylim(-1.0, 6.0)
        self.ax.set_xlabel(r'$x (m)$')
        self.ax.set_ylabel(r'$y (m)$')
        self.ax.grid(True)

        self.plot_goals()
        self.plot_obstacles()
        self.ax.add_collection(self.human_collection)
        self.ax.add_collection(self.vel_collection)

        return (self.human_collection, self.vel_collection)        


    def animate(self, total_time=15):
        """Main method to create animation"""
        ani = mpl_animation.FuncAnimation(
            self.figure,
            init_func=self.animation_init,
            func=self.animation_update,
            frames=total_time,
            blit=True,
        )
        plt.close()
        return ani