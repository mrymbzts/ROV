import matplotlib.pyplot as plt 
from simple_pid import PID
import time

#Setting the PİD values
#Proportional(P): Shows current error, (amount)
#Integral(I): Accumulates past errors, (size)
#Derivative(D): Predicts future error, (rate of change)
pid = PID(Kp=2.0, Ki=0.5, Kd=1.0, setpoint=50.0) #target depth is 50m

#Max and min power to the ROV's motors
pid.output_limits = (-500, 500) #we assume it's (-500 to 500)

#Virtual environment
current_depth = 0.0 #starts from water surface
velocity = 0.0 #initial velocity 
mass = 15 #total mass of the rov (kg)
friction = 0.7 #force of friction of water (must be calculated)
thrust_multiplier = 0.1 #for converting (e.g. 500 PWM = 50 Newtons)

#some data lists for graphic
time_list = []
depth_list = []
target_list = []

#Simulation loop
dt = 0.1
total_time = 80.0 #we set the timer to 80 seconds
elapsed_time = 0.0

while elapsed_time < total_time :
    # PID calculation:PWM signal between -500 and 500.
    correction = pid(current_depth) 
    # Add this correction to the neutral throttle (1500)
    motor_pwm = 1500 + correction


    # 1500 PWM is zero thrust point
    # first pull the signal to the center then convert it to force.
    applied_force = (motor_pwm - 1500) * thrust_multiplier

    #F = m.a 
    #measuring total force: 1. motors 2. friction of water
    total_force = applied_force - (velocity * friction)
    acceleration = total_force / mass

    #Updating speed and new depth
    velocity += acceleration * dt
    current_depth += velocity * dt

    #Saving the dataset for graphic 
    time_list.append(elapsed_time)
    depth_list.append(current_depth)
    target_list.append(50.0)

    elapsed_time += dt

#Creating the plot
plt.figure(figsize=(10,5))
plt.plot(time_list, depth_list, label= 'Current Depth (ROV)', color='blue', linewidth=2 )
plt.plot(time_list, target_list, label= 'Target Depth (50m)', color='red', linestyle='--' )
plt.title('ROV PID Depth Control Simulation (PWM)')
plt.xlabel('Time (Second)')
plt.ylabel('Depth (Meter)')
plt.legend()
plt.grid(True)
plt.show()

