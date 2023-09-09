import pybullet as p
import time
import numpy as np

# 2. Define Rocket Parameters
totalMass = 1
dryMass = 0.906
burnTime = 3.4
totalImpulse = 49.6
propellantMass = 0.064

# 3. Compute Rocket Dynamics
## Calculate Average Thrust and Mass Flow Rate
averageThrust = totalImpulse/burnTime
massFlowRate = propellantMass/burnTime

t = np.linspace(0, 10, 100, False)

## Calculate Thrust as a function of Time
index = (np.abs(t - burnTime)).argmin() + 1
thrust = np.append(np.repeat(averageThrust, index), np.repeat(0, len(t) - index))

## Calculate Mass as a function of Time
mass = np.append(np.repeat(totalMass, index) - t[0:index] * massFlowRate, np.repeat(dryMass, len(t) - index))

## Determine Acceleration, Velocity, and Displacement over Time
acceleration = thrust/mass - 9.81

# 1. Initialize PyBullet
p.connect(p.GUI)
p.setGravity(0, -9.81, 0)  # Gravity in negative y-direction
p.setTimeStep(0.1)  # Simulation time step

# 2. Load Environment and Rocket
planeID = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, planeID)

rocketShape = p.createCollisionShape(p.GEOM_CAPSULE, radius=0.05, height=0.3, collisionFramePosition=[0, 0.15, 0])
rocketMass = totalMass
rocketID = p.createMultiBody(rocketMass, rocketShape)

# 3. Simulation Loop
simulation_duration = len(t) 
current_time = 0

while current_time < simulation_duration:
    t_index = int(current_time / 0.1)
    if t_index < len(thrust) and thrust[t_index] > 0:
        # Apply the thrust along the y-axis
        p.applyExternalForce(rocketID, -1, [0, thrust[t_index], 0], [0, 0, 0], p.WORLD_FRAME)

    p.stepSimulation()
    time.sleep(0.01)  # Slow down the loop for visualization
    current_time += 0.1

p.disconnect()
