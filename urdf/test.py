import pybullet as p 
import pybullet_data 
p.connect(p.DIRECT) 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
p.setGravity(0,0,-9.81) 
r = p.loadURDF('wcr/urdf/wcr.urdf', basePosition=[0,0,1]) 
for i in range(50): 
    p.stepSimulation() 
pos, orn = p.getBasePositionAndOrientation(r) 
euler = p.getEulerFromQuaternion(orn) 
print('Position:', [round(x,3) for x in pos]) 
print('Euler:', [round(x,3) for x in euler]) 
p.disconnect() 
