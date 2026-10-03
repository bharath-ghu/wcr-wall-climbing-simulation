"""
Wall Climbing Robot — Simulation v4
=====================================
Fixes:
  - Back wall detection completely rewritten (approach from +X side)
  - Front/rear fans tilt independently (VertiGo style)
  - Propeller speed increased to real values
  - Wheel spin direction tied to movement correctly
  - HUD uses replaceItemUniqueId (stable, no flicker)
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# ─────────────────────────────────────────────
# CONNECT
# ─────────────────────────────────────────────
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)
p.resetDebugVisualizerCamera(
    cameraDistance=6.0, cameraYaw=50,
    cameraPitch=-20, cameraTargetPosition=[3.0, 0, 1.5])

# ─────────────────────────────────────────────
# WALL & ENVIRONMENT
# ─────────────────────────────────────────────
p.loadURDF("plane.urdf", [0, 0, 0])

WALL_X         = 4.0
WALL_Y_HALF    = 2.5
WALL_Z_TOP     = 4.0
WALL_T         = 0.1
WALL_FRONT_X   = WALL_X - WALL_T/2
WALL_BACK_X    = WALL_X + WALL_T/2

wall_col = p.createCollisionShape(p.GEOM_BOX,
    halfExtents=[WALL_T/2, WALL_Y_HALF, WALL_Z_TOP/2])
wall_vis = p.createVisualShape(p.GEOM_BOX,
    halfExtents=[WALL_T/2, WALL_Y_HALF, WALL_Z_TOP/2],
    rgbaColor=[0.75, 0.70, 0.60, 1.0])
wall_id = p.createMultiBody(0, wall_col, wall_vis,
    basePosition=[WALL_X, 0, WALL_Z_TOP/2])
p.changeDynamics(wall_id, -1, lateralFriction=1.0, restitution=0.0)

# Draw grid on both faces
def draw_grid(face_x, direction, color):
    for z in np.arange(0.5, WALL_Z_TOP, 0.5):
        p.addUserDebugLine([face_x, -WALL_Y_HALF, z],
                           [face_x,  WALL_Y_HALF, z], color, 1)
    for y in np.arange(-WALL_Y_HALF, WALL_Y_HALF+0.5, 0.5):
        p.addUserDebugLine([face_x, y, 0],
                           [face_x, y, WALL_Z_TOP], color, 1)

draw_grid(WALL_FRONT_X - 0.01, -1, [0.6, 0.55, 0.45])
draw_grid(WALL_BACK_X  + 0.01, +1, [0.5, 0.45, 0.35])

# ─────────────────────────────────────────────
# ROBOT
# ─────────────────────────────────────────────
robot = p.loadURDF("wcr/urdf/wcr.urdf",
    basePosition=[0.0, 0.0, 0.35], useFixedBase=False)

for link, col in {
    -1:[0.2,0.2,0.2,1], 0:[0.1,0.1,0.1,1], 1:[0.1,0.1,0.1,1],
     2:[0.1,0.1,0.1,1], 3:[0.1,0.1,0.1,1], 4:[0.8,0.3,0.1,1],
     5:[0.6,0.6,0.6,1], 6:[0.2,0.6,1.0,1], 7:[0.8,0.3,0.1,1],
     8:[0.6,0.6,0.6,1], 9:[0.2,0.6,1.0,1]}.items():
    try: p.changeVisualShape(robot, link, rgbaColor=col)
    except: pass

p.changeDynamics(robot, -1, lateralFriction=0.9,
    linearDamping=0.1, angularDamping=0.9, rollingFriction=0.01)

# ─────────────────────────────────────────────
# CONSTANTS
# ─────────────────────────────────────────────
ROBOT_R      = 0.35
OFFSET_FLOOR = 0.35
# Robot surface position on each wall face
FRONT_RX     = WALL_FRONT_X - ROBOT_R
BACK_RX      = WALL_BACK_X  + ROBOT_R

DRIVE_SPEED  = 0.04
CLIMB_SPEED  = 0.03
TURN_SPEED   = 0.04
PRESS_FORCE  = 20.0

# Fan speeds (rad/s)
# Real VertiGo ~10000RPM = 1047 rad/s, scaled for PyBullet
FAN_STOP     = 0.0
FAN_IDLE     = 80.0    # slow spin when on floor moving
FAN_WALL     = 900.0   # full wall climbing speed

# Fan tilt angles (radians, on Z axis per check.py)
# On floor: fans flat (0)
# On front wall: rear fan presses into wall (pi/2),
#                front fan tilts upward (pi/4) — VertiGo style
# On back wall: opposite directions
TILT_FLOOR       = 0.0
TILT_REAR_WALL   = math.pi / 2     # 90° — presses into wall
TILT_FRONT_WALL  = math.pi / 4     # 45° — upward component
TILT_SPEED       = 0.12            # interpolation per frame

# ─────────────────────────────────────────────
# STATE: 0=Floor  1=Front Wall  2=Back Wall
# ─────────────────────────────────────────────
state  = 0
rx, ry, rz   = 0.0, 0.0, OFFSET_FLOOR
ryaw, rpitch = 0.0, 0.0

# Independent tilt for front (f_or=4) and rear (r_or=7)
tilt_front_cur = 0.0
tilt_rear_cur  = 0.0
tilt_front_tgt = 0.0
tilt_rear_tgt  = 0.0
fan_spd        = FAN_STOP

# ─────────────────────────────────────────────
# JOINT HELPERS
# ─────────────────────────────────────────────
# Joint map (from check.py):
# 0:w1 1:w2 2:w3 3:w4  (Z axis, wheels)
# 4:f_or (Z axis, front tilt)
# 5:f_il (X axis, front inner)
# 6:f   (Y axis, front blade)
# 7:r_or (Z axis, rear tilt)
# 8:r_il (X axis, rear inner)
# 9:r   (Y axis, rear blade)

def vel_ctrl(joints, vel, force=100):
    for j in joints:
        p.setJointMotorControl2(robot, j,
            p.VELOCITY_CONTROL, targetVelocity=vel, force=force)

def pos_ctrl(joint, angle, force=5, max_vel=6.0):
    p.setJointMotorControl2(robot, joint,
        p.POSITION_CONTROL, targetPosition=angle,
        force=force, maxVelocity=max_vel)

def lerp(cur, tgt, speed):
    diff = tgt - cur
    return cur + diff * speed

# ─────────────────────────────────────────────
# HUD SETUP
# ─────────────────────────────────────────────
# Place HUD text in world space near top of wall
HX, HY, HZ = -2.0, -3.8, WALL_Z_TOP

t_title = p.addUserDebugText("■ WCR SIMULATION",
    [HX, HY, HZ+1.0], [1,1,1], 1.4, lifeTime=0)
t_ctrl  = p.addUserDebugText("↑↓ Climb/Drive   ←→ Turn/Slide   Q Quit",
    [HX, HY, HZ+0.55], [0.9,0.9,0.2], 0.9, lifeTime=0)
t_surf  = p.addUserDebugText("Surface: FLOOR",
    [HX, HY, HZ+0.1], [0.3,1.0,0.3], 1.1, lifeTime=0)
t_pos   = p.addUserDebugText("Pos  X:+0.00  Y:+0.00  Z:+0.00",
    [HX, HY, HZ-0.35], [0.3,1.0,0.3], 1.0, lifeTime=0)
t_thr   = p.addUserDebugText("Thrust: 0.0 N",
    [HX, HY, HZ-0.8], [0.2,0.8,1.0], 1.0, lifeTime=0)
t_fan   = p.addUserDebugText("Front fan: 0.0°  Rear fan: 0.0°  Speed: 0 rad/s",
    [HX, HY, HZ-1.25], [0.2,0.8,1.0], 1.0, lifeTime=0)

SURF_NAMES = {0:"FLOOR", 1:"FRONT WALL", 2:"BACK WALL"}

def update_hud(thrust):
    p.addUserDebugText(f"Surface: {SURF_NAMES[state]}",
        [HX,HY,HZ+0.1], [0.3,1.0,0.3], 1.1, lifeTime=0.5,
        replaceItemUniqueId=t_surf)
    p.addUserDebugText(
        f"Pos  X:{rx:+.2f}  Y:{ry:+.2f}  Z:{rz:+.2f}",
        [HX,HY,HZ-0.35], [0.3,1.0,0.3], 1.0, lifeTime=0.5,
        replaceItemUniqueId=t_pos)
    p.addUserDebugText(f"Thrust: {thrust:.1f} N",
        [HX,HY,HZ-0.8], [0.2,0.8,1.0], 1.0, lifeTime=0.5,
        replaceItemUniqueId=t_thr)
    p.addUserDebugText(
        f"Front fan: {math.degrees(tilt_front_cur):+.1f}°  "
        f"Rear fan: {math.degrees(tilt_rear_cur):+.1f}°  "
        f"Speed: {fan_spd:.0f} rad/s  (~{fan_spd*9.55:.0f} RPM)",
        [HX,HY,HZ-1.25], [0.2,0.8,1.0], 1.0, lifeTime=0.5,
        replaceItemUniqueId=t_fan)

# ─────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────
print("\n── WCR Simulation v4 ──")
print("  Drive toward front of wall to climb front face.")
print("  Drive AROUND the wall (+X side) to climb back face.")
print("  Q to quit.\n")

step   = 0
thrust = 0.0

while p.isConnected():
    try: keys = p.getKeyboardEvents()
    except: break

    if ord('q') in keys and keys[ord('q')] & p.KEY_IS_DOWN:
        break

    fwd   = p.B3G_UP_ARROW    in keys and keys[p.B3G_UP_ARROW]    & p.KEY_IS_DOWN
    back  = p.B3G_DOWN_ARROW   in keys and keys[p.B3G_DOWN_ARROW]  & p.KEY_IS_DOWN
    left  = p.B3G_LEFT_ARROW   in keys and keys[p.B3G_LEFT_ARROW]  & p.KEY_IS_DOWN
    right = p.B3G_RIGHT_ARROW  in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN
    moving = fwd or back or left or right

    # ── FLOOR ───────────────────────────────────
    if state == 0:
        rpitch = 0.0
        rz     = OFFSET_FLOOR
        thrust = 0.0
        tilt_front_tgt = TILT_FLOOR
        tilt_rear_tgt  = TILT_FLOOR

        if left:  ryaw += TURN_SPEED
        if right: ryaw -= TURN_SPEED

        d = DRIVE_SPEED if fwd else (-DRIVE_SPEED if back else 0.0)
        rx += d * math.cos(ryaw)
        ry += d * math.sin(ryaw)

        # Front wall — robot approaching from left (rx increasing)
        if (rx >= FRONT_RX and abs(ry) <= WALL_Y_HALF
                and fwd and math.cos(ryaw) > 0.3):
            state  = 1
            rx     = FRONT_RX
            ryaw   = 0.0
            rpitch = -math.pi / 2
            print("[→] Floor → Front Wall")

        # Back wall — robot approaching from right (rx decreasing toward back)
        # Robot must have gone past the wall (rx > WALL_BACK_X)
        # and be moving in -X direction (ryaw near ±pi)
        if (rx <= BACK_RX and rx >= WALL_BACK_X
                and abs(ry) <= WALL_Y_HALF
                and fwd and math.cos(ryaw) < -0.3):
            state  = 2
            rx     = BACK_RX
            ryaw   = math.pi
            rpitch = math.pi / 2
            print("[→] Floor → Back Wall")

        fan_spd  = FAN_IDLE if moving else FAN_STOP
        wheel_v  = 15.0 if fwd else (-15.0 if back else 0.0)
        vel_ctrl([0,1,2,3], wheel_v)
        vel_ctrl([6], fan_spd)        # front blade
        vel_ctrl([9], -fan_spd)       # rear blade (counter-rotate)
        vel_ctrl([5,8], 1.0 if moving else 0.0)

    # ── FRONT WALL ──────────────────────────────
    elif state == 1:
        rpitch = -math.pi / 2
        rx     = FRONT_RX
        thrust = PRESS_FORCE
        # VertiGo: rear fan presses into wall, front fan tilts upward
        tilt_front_tgt = TILT_FRONT_WALL   # 45°
        tilt_rear_tgt  = TILT_REAR_WALL    # 90°

        if fwd:   rz += CLIMB_SPEED
        if back:  rz -= CLIMB_SPEED
        if left:  ry += CLIMB_SPEED
        if right: ry -= CLIMB_SPEED

        ry = max(-WALL_Y_HALF+ROBOT_R, min(WALL_Y_HALF-ROBOT_R, ry))
        rz = max(OFFSET_FLOOR, min(WALL_Z_TOP-ROBOT_R, rz))

        if rz <= OFFSET_FLOOR and back:
            state  = 0
            rx     = FRONT_RX - 0.15
            ryaw   = 0.0
            print("[→] Front Wall → Floor")

        pos, _ = p.getBasePositionAndOrientation(robot)
        p.applyExternalForce(robot, -1,
            [PRESS_FORCE, 0, 0], pos, p.WORLD_FRAME)

        fan_spd = FAN_WALL
        wv = 15.0 if fwd else (-15.0 if back else 0.0)
        vel_ctrl([0,1,2,3], wv)
        vel_ctrl([6],  fan_spd)
        vel_ctrl([9], -fan_spd)
        vel_ctrl([5,8], 3.0)

    # ── BACK WALL ───────────────────────────────
    elif state == 2:
        rpitch = math.pi / 2
        rx     = BACK_RX
        thrust = PRESS_FORCE
        # Opposite tilt direction for back wall
        tilt_front_tgt = -TILT_FRONT_WALL
        tilt_rear_tgt  = -TILT_REAR_WALL

        if fwd:   rz += CLIMB_SPEED
        if back:  rz -= CLIMB_SPEED
        if left:  ry -= CLIMB_SPEED
        if right: ry += CLIMB_SPEED

        ry = max(-WALL_Y_HALF+ROBOT_R, min(WALL_Y_HALF-ROBOT_R, ry))
        rz = max(OFFSET_FLOOR, min(WALL_Z_TOP-ROBOT_R, rz))

        if rz <= OFFSET_FLOOR and back:
            state  = 0
            rx     = BACK_RX + 0.15
            ryaw   = math.pi
            print("[→] Back Wall → Floor")

        pos, _ = p.getBasePositionAndOrientation(robot)
        p.applyExternalForce(robot, -1,
            [-PRESS_FORCE, 0, 0], pos, p.WORLD_FRAME)

        fan_spd = FAN_WALL
        wv = 15.0 if fwd else (-15.0 if back else 0.0)
        vel_ctrl([0,1,2,3], wv)
        vel_ctrl([6],  fan_spd)
        vel_ctrl([9], -fan_spd)
        vel_ctrl([5,8], 3.0)

    # ── SMOOTH FAN TILT (independent front/rear) ─
    tilt_front_cur = lerp(tilt_front_cur, tilt_front_tgt, TILT_SPEED)
    tilt_rear_cur  = lerp(tilt_rear_cur,  tilt_rear_tgt,  TILT_SPEED)
    pos_ctrl(4, tilt_front_cur)   # f_or — front tilt
    pos_ctrl(7, tilt_rear_cur)    # r_or — rear tilt

    # ── APPLY TRANSFORM ─────────────────────────
    p.resetBasePositionAndOrientation(robot,
        [rx, ry, rz],
        p.getQuaternionFromEuler([0, rpitch, ryaw]))

    # ── HUD ─────────────────────────────────────
    if step % 15 == 0:
        update_hud(thrust)

    p.stepSimulation()
    time.sleep(1.0/60.0)
    step += 1

try: p.disconnect()
except: pass
print("Done.")
