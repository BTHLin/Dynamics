import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
global g
g = np.array([0,0,-9.81])


class Dimension_3d():
    def __init__(self):
        self.x = []
        self.y = []
        self.z = []

class Logging():
    def __init__(self):
        dim = Dimension_3d()
        self.time = []
        self.pos = Dimension_3d()
        self.vel = Dimension_3d()
        self.mtm = Dimension_3d()

class Coefficient():
    def __init__(self):
        self.restitution_ball_2_ground = 0
        self.restitution_ball_2_ball = 0

class Object():
    def __init__(self):
        self.m = 0.  # mass - scaler
        self.p = np.array([0.,0.,0.]) # momentum - vector
        self.v = np.array([0.,0.,0.]) # velocity - vector
        self.r = np.array([0.,0.,0.]) # postion - vector
        self.a = np.array([0.,0.,0.]) # acceleration - vector
        self.d = 0. # diameter - scalar

class Tools():
    def distance_btw_mass(self, r1, r2):
        dx = abs(r1[0]-r2[0])
        dy = abs(r1[1]-r2[1])
        dz = abs(r1[2]-r2[2])
        return np.sqrt(dx**2 + dy**2 + dz**2)

class Trajectory():
    def projectile(self, ball, log, Tt, dt):
        t = 0 

        while t< Tt and ball.r[2] >= 0:
            ball.p += ball.m * ball.a * dt# update momentum
            ball.v += ball.a * dt # update velocity
            ball.r += ball.p/ball.m * dt  # update position
            t += dt

            # logging
            log.time.append(t)
            log.pos.x.append(ball.r[0])
            log.pos.y.append(ball.r[1])
            log.pos.z.append(ball.r[2])
            log.vel.x.append(ball.v[0])
            log.vel.y.append(ball.v[1])
            log.vel.z.append(ball.v[2])
    
    def bouncing_ball(self, ball, log, Tt, dt, coef):
        t = 0
        
        while t < Tt:

            ball.p += ball.m * ball.a * dt # update momentum
            ball.r += ball.p/ball.m * dt # update position
            ball.v += ball.a * dt # update velocity
            
            if ball.r[2] < 0:
                ball.v[2] = -coef.restitution_ball_2_ground * ball.v[2]
                ball.p = ball.m * ball.v
                ball.r[2] = 0

            # logging
            log.time.append(t)
            log.pos.x.append(ball.r[0])
            log.pos.y.append(ball.r[1])
            log.pos.z.append(ball.r[2])
            log.vel.x.append(ball.v[0])
            log.vel.y.append(ball.v[1])
            log.vel.z.append(ball.v[2])

            t += dt

    def two_bouncing_balls(self, ball_1, ball_2, log_1, log_2, Tt, dt, coef):
        t = 0
        tool = Tools()

        while t < Tt:
            # update ball 1
            ball_1.p += ball_1.m * ball_1.a * dt # update momentum
            ball_1.r += ball_1.p/ball_1.m * dt # update position
            ball_1.v += ball_1.a * dt # update velocity
            
            # update ball 2
            ball_2.p += ball_2.m * ball_2.a * dt # update momentum
            ball_2.r += ball_2.p/ball_2.m * dt # update position
            ball_2.v += ball_2.a * dt # update velocity

            # update timestamp
            t += dt

            # ball_1 collided with the ground
            if ball_1.r[2] <= 0:
                ball_1.v = -coef.restitution_ball_2_ground * ball_1.v
                ball_1.p = ball_1.m * ball_1.v
                ball_1.r[2] = 0
            
            # ball_2 collided with the ground
            if ball_2.r[2] <= 0:
                ball_2.v = -coef.restitution_ball_2_ground * ball_2.v
                ball_2.p = ball_2.m * ball_2.v
                ball_2.r[2] = 0

            # ball_1 collides with ball_2
            if tool.distance_btw_mass(ball_1.r, ball_2.r) <= 0.5*(ball_1.d + ball_2.d):
                v1_restoration = - ball_1.v * coef.restitution_ball_2_ball
                ball_2.v = (3*ball_1.m-ball_2.m)/(ball_1.m+ball_2.m) * v1_restoration
                ball_1.v = ((ball_1.m *v1_restoration + ball_2.p) - ball_2.m*ball_2.v) / ball_1.m
                ball_1.p = ball_1.m * ball_1.v
                ball_2.p = ball_2.m * ball_2.v
                ball_2.r[2] = ball_1.r[2] + 0.5*(ball_1.d+ball_2.d)
                
            # logging
            log_1.time.append(t)
            log_1.pos.x.append(ball_1.r[0])
            log_1.pos.y.append(ball_1.r[1])
            log_1.pos.z.append(ball_1.r[2])
            log_1.vel.x.append(ball_1.v[0])
            log_1.vel.y.append(ball_1.v[1])
            log_1.vel.z.append(ball_1.v[2]) 
            
            log_2.time.append(t)
            log_2.pos.x.append(ball_2.r[0])
            log_2.pos.y.append(ball_2.r[1])
            log_2.pos.z.append(ball_2.r[2])
            log_2.vel.x.append(ball_2.v[0])
            log_2.vel.y.append(ball_2.v[1])
            log_2.vel.z.append(ball_2.v[2]) 


def plot(log):
    plt.plot(log.pos.x,log.pos.y)
    plt.show()
    
def animate_position(frame, ax, line_1, line_2, log_1, log_2, circle_1, circle_2, height_text):
    """For each frame, advance the animation to the new position, pos."""
    x = log_1.pos.x[0:frame]
    z = log_1.pos.z[0:frame]
    line_1.set_data(x,z)

    x = log_2.pos.x[0:frame]
    z = log_2.pos.z[0:frame]
    line_2.set_data(x,z)
    
    # update circle
    circle_1.set_center((log_1.pos.x[frame],log_1.pos.z[frame]))
    circle_2.set_center((log_2.pos.x[frame],log_2.pos.z[frame]))
    
    # update text
    height_text.set_text(f'Time: {log_1.time[frame]:.1f} s')

def animate_height_vs_time(frame, ax, line_1, line_2, log_1, log_2, circle_1, circle_2):
    """For each frame, advance the animation to the new position, pos."""
    x = log_1.time[0:frame]
    z1 = log_1.pos.z[0:frame]
    z2 = log_2.pos.z[0:frame]
    line_1.set_data(x,z1)
    line_2.set_data(x,z2)
    
    # update circle
    circle_1.set_center((log_1.time[frame],log_1.pos.z[frame]))
    circle_2.set_center((log_2.time[frame],log_2.pos.z[frame]))
    
    
def plot_position(log_1, log_2, ball_1, ball_2):
    #  Set up a new Figure, with equal aspect ratio so the ball appears round.
    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    # initialize the plot
    xmax = max(max(log_1.pos.x),max(log_2.pos.x))
    zmax = max(max(log_1.pos.z),max(log_2.pos.z))
    ax.set_xlim(- max(xmax,zmax), max(xmax,zmax))
    ax.set_ylim(0, max(xmax,zmax))
 
    line_1, = ax.plot([], [], '--',lw=2, color='blue', alpha=.8) # same as line = ax.plot([])
    line_2, = ax.plot([], [], '-.',lw=2, color='red', alpha=.8) # same as line = ax.plot([])
    circle_1 = plt.Circle((0, 0), radius=0.5*ball_1.d, color='blue', alpha=.3)
    circle_2 = plt.Circle((0, 0), radius=0.5*ball_2.d, color='red', alpha=.3)
    height_text = ax.text((max(max(log_1.pos.x),max(log_2.pos.x)))*0.7, max(max(log_1.pos.z),max(log_2.pos.x))*0.8, f'Time: {log_1.time[0]:.1f} s')
    ax.add_patch(circle_1) # display ball_1
    ax.add_patch(circle_2) # display ball_2
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Height (m)")
    ax.grid()

    anim = animation.FuncAnimation(fig, animate_position, frames = min(len(log_1.time), len(log_2.time)), fargs=(ax, line_1, line_2, log_1, log_2, circle_1, circle_2, height_text), interval=1)
    dir_animation = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'animations/bouncing_ball.gif')
    # anim.save(dir_animation, writer='Pillow', fps=30)
    plt.show()

def plot_height_vs_time(log_1, log_2, ball_1, ball_2, Tt):
    #  Set up a new Figure, with equal aspect ratio so the ball appears round.
    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    # initialize the plot
    zmax = max(max(log_1.pos.z),max(log_2.pos.z))
    ax.set_xlim(0, Tt)
    ax.set_ylim(0, max(Tt,zmax))
 
    line_1, = ax.plot([], [], '--',lw=2, color='blue', alpha=.8) # same as line = ax.plot([])
    line_2, = ax.plot([], [], '-.',lw=2, color='red', alpha=.8) # same as line = ax.plot([])
    circle_1 = plt.Circle((0, 0), radius=0.5*ball_1.d, color='blue', alpha=.3)
    circle_2 = plt.Circle((0, 0), radius=0.5*ball_2.d, color='red', alpha=.3)

    ax.add_patch(circle_1) # display ball_1
    ax.add_patch(circle_2) # display ball_2
    ax.set_xlabel("Time (t)")
    ax.set_ylabel("Height (m)")
    ax.grid()

    anim = animation.FuncAnimation(fig, animate_height_vs_time, frames = min(len(log_1.time), len(log_2.time)), fargs=(ax, line_1, line_2, log_1, log_2, circle_1, circle_2), interval=1)
    dir_animation = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'animations/2_bouncing_balls.gif')
    anim.save(dir_animation, writer='Pillow', fps=30)
    plt.show()
    

def main():
    # Physics Model
    Traj = Trajectory()
    coef = Coefficient()

    # logging
    log_1 = Logging() # for ball 1
    log_2 = Logging() # for ball 2
    
    # initial condition
    theta = np.deg2rad(90) 
    dz = 0.1 # distance between the balls

    ball_1 = Object()
    ball_1.m = 0.5 # mass (Kg)
    ball_1.d = 0.5 # diameter = 0.5 m
    ball_1.v = 0. * np.array([math.cos(theta), 0, math.sin(theta)])
    ball_1.p = ball_1.m * ball_1.v # inital momentum
    ball_1.r = np.array([0,0,9.]) # intial position [x, y, z]
    ball_1.a = g # initial acceleration

    ball_2 = Object()
    ball_2.m = .3 # mass (kg)
    ball_2.d = 0.1 # diameter (m)
    ball_2.v = 0 * np.array([math.cos(theta), 0, math.sin(theta)])
    ball_2.p = ball_2.m * ball_2.v # inital momentum
    ball_2.r = np.array([0,0,ball_1.r[2]+0.5*(ball_1.d+ball_2.d)+dz]) # intial position [x, y, z]
    ball_2.a = g # initial acceleration
    

    # time control
    Tt = 10 # total time (s)
    dt = 0.01

    # simulation
    
    ## simple projectile
    # Traj.projectile(ball_1, log, Tt, dt)
    
    ## bouncing balls
    # restitution_ball_2_ground = 0.85
    # Traj.bouncing_ball(ball_1, ball_2, log_1, log_2, Tt, dt, coef)

    ## bouncing balls
    coef.restitution_ball_2_ground = 0.85
    coef.restitution_ball_2_ball = 1.
    Traj.two_bouncing_balls(ball_1, ball_2, log_1, log_2, Tt, dt, coef)


    # result visualization
    # plot_position(log_1, log_2, ball_1, ball_2)
    plot_height_vs_time(log_1, log_2, ball_1, ball_2, Tt)


if __name__ == "__main__":
    main()