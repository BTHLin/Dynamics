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
        self.restitution = 0

class Object():
    def __init__(self):
        self.m = 0.  # mass - scaler
        self.p = np.array([0.,0.,0.]) # momentum - vector
        self.v = np.array([0.,0.,0.]) # velocity - vector
        self.r = np.array([0.,0.,0.]) # postion - vector
        self.a = np.array([0.,0.,0.]) # acceleration - vector

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
        
        while ball.r[0] <= 16:
            ball.p += ball.m * ball.a * dt # update momentum
            ball.r += ball.p/ball.m * dt # update position
            ball.v += ball.a * dt # update velocity
            t += dt
            if ball.r[2] < 0:
                ball.v[2] = -coef.restitution * ball.v[2]
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
            


def plot(log):
    plt.plot(log.pos.x,log.pos.y)
    plt.show()
    
def animate(frame, line, ax, log, ball, height_text):
    """For each frame, advance the animation to the new position, pos."""
    x = log.pos.x[0:frame]
    z = log.pos.z[0:frame]
    line.set_data(x,z)
    
    ball.set_center((log.pos.x[frame],log.pos.z[frame]))
    height_text.set_text(f'Time: {log.time[frame]:.1f} s')
    
    
def plot_simulation(log, Tt, dt):
    #  Set up a new Figure, with equal aspect ratio so the ball appears round.
    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    # initialize the plot
    T0 = log.time[0]

    ax.set_xlim(0, max(max(log.pos.x),max(log.pos.z)))
    ax.set_ylim(0, max(max(log.pos.x),max(log.pos.z)))
 
    line, = ax.plot([], [], '--',lw=2) # same as line = ax.plot([])
    ball = plt.Circle((0, 0), 0.2)
    height_text = ax.text((max(log.pos.x))*0.7, max(log.pos.z)*0.8, f'Time: {T0:.1f} s')
    ax.add_patch(ball) # display the ball
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.grid()

    anim = animation.FuncAnimation(fig, animate, frames = len(log.time), fargs=(line, ax, log, ball, height_text), interval=1)
    dir_animation = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'animations/bouncing_ball.gif')
    anim.save(dir_animation, writer='Pillow', fps=30)
    plt.show()
    

def main():
    # Physics Model
    Traj = Trajectory()
    coef = Coefficient()

    # logging
    log = Logging()
    
    # initial condition
    theta = np.deg2rad(85) 
    ball = Object()
    ball.m = 0.2
    ball.v = 10 * np.array([math.cos(theta), 0, math.sin(theta)])
    ball.p = ball.m * ball.v # inital momentum
    ball.r = np.array([0,0,5.]) # intial position [x, y, z]
    ball.a = g # initial acceleration

    # time control
    Tt = 10 # total time (s)
    dt = 0.01

    # simulation
    
    ## simple projectile
    # Traj.projectile(ball, log, Tt, dt)
    
    ## bouncing ball
    coef.restitution = 0.85
    Traj.bouncing_ball(ball, log, Tt, dt, coef)

    # result visualization
    # plot(log)
    plot_simulation(log, Tt, dt)


if __name__ == "__main__":
    main()