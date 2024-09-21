import pybullet as p
import time
import math
import numpy as np
import random


############################################### Environment Setup ####################################################
p.connect(p.GUI)

p.resetSimulation()

p.setGravity(0, 0, -10)
useRealTimeSim = 0

p.setRealTimeSimulation(useRealTimeSim)  # either this

# load plane
track = p.loadURDF("./plane/plane.urdf")
# load car
car = p.loadURDF("./f10_racecar/racecar_differential.urdf", [0, 0, 0])


# load obstacles, in this projects, we used six cube as obstacles

def random_obstacles():
    np.random.seed()
    xy_position = [0, 0]
    xy_position_float = np.random.rand(2)
    x_poistion_range = np.random.randint(1, 10)
    y_poistion_range = np.random.randint(1, 10)

    xy_position[0] = xy_position_float[0] + x_poistion_range
    xy_position[1] = xy_position_float[1] + y_poistion_range

    np.asarray(xy_position)
    position = np.append(xy_position, 0.2)
    return position


# Total eight different cubes. The cube's position is changing for each time.
total_cubes_number = 10
data_path = 'data/'
cube_list = []
cube_name_list = ['cube_black/marble_cube_black.urdf',
                  'cube_green/marble_cube_green.urdf',
                  'cube/marble_cube.urdf']

for i in range(total_cubes_number):
    cube_list.append(random.choice(cube_name_list))

# Only two objects are loaded in stable positon.
# You need to design URDF files to load objects, otherwise the code will not excute.
cube_stable_position_1 = p.loadURDF('./cube_black/marble_cube_black.urdf',(5,5,0.2))
cube_stable_position_2 = p.loadURDF('./cube/marble_cube.urdf',(3,3,0.2))

# 10 objects are loaed in random positions.
cube_1_position = random_obstacles()
cube_1 = p.loadURDF('./'+cube_list[0],cube_1_position)

cube_2_position = random_obstacles()
cube_2 = p.loadURDF('./'+cube_list[1],cube_2_position)

cube_3_position = random_obstacles()
cube_3 = p.loadURDF('./'+cube_list[2],cube_3_position)

cube_4_position = random_obstacles()
cube_4 = p.loadURDF('./'+cube_list[3],cube_4_position)

cube_5_position = random_obstacles()
cube_5 = p.loadURDF('./'+cube_list[4],cube_5_position)

cube_6_position = random_obstacles()
cube_6 = p.loadURDF('./'+cube_list[5],cube_6_position)

cube_7_position = random_obstacles()
cube_7 = p.loadURDF('./'+cube_list[6],cube_7_position)

cube_8_position = random_obstacles()
cube_8 = p.loadURDF('./'+cube_list[7],cube_8_position)

cube_9_position = random_obstacles()
cube_9 = p.loadURDF('./'+cube_list[8],cube_9_position)

cube_10_position = random_obstacles()
cube_10 = p.loadURDF('./'+cube_list[9],cube_10_position)

for wheel in range(p.getNumJoints(car)):
    print("joint[", wheel, "]=", p.getJointInfo(car, wheel))
    p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    p.getJointInfo(car, wheel)

wheels = [8, 15]
print("----------------")

c = p.createConstraint(car, 9, car, 11, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 10, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 9, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 16, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 16, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 17, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 1, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)
c = p.createConstraint(car, 3, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

steering = [0, 2]

hokuyo_joint = 4


replaceLines = True

numRays = 100
rayFrom = []
rayTo = []
rayIds = []
rayHitColor = [1, 0, 0]
rayMissColor = [0, 1, 0]
rayLen = 8
rayStartLen = 0.25
for i in range(numRays):
    rayFrom.append([rayStartLen * math.sin(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays),
                    rayStartLen * math.cos(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays), 0])
    rayTo.append([rayLen * math.sin(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays),
                  rayLen * math.cos(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays), 0])
    if (replaceLines):
        rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, parentObjectUniqueId=car,
                                         parentLinkIndex=hokuyo_joint))
    else:
        rayIds.append(-1)

frame = 0
lineId = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
lineId2 = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
lineId3 = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
print("lineId=", lineId)
lastTime = time.time()
lastControlTime = time.time()
lastLidarTime = time.time()
time.sleep(0.03)
def drive_the_mobile(carPos, carOrn, turn_angle,final_goal_pos):

    steeringAngle = 1 * turn_angle
    if steeringAngle > 1:
        steeringAngle = 1
    if steeringAngle < -1:
        steeringAngle = -1

    distance = np.sqrt(np.square(final_goal_pos[0] - carPos[0]) + np.square(final_goal_pos[1] - carPos[1]))

    stop_threshold = 1
    if distance < stop_threshold:
        targetVelocity = 0
    else:
        targetVelocity = 25

    for wheel in wheels:
        p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=targetVelocity, force=maxForce)

    for steer in steering:
        print("steeringAngle", np.degrees(steeringAngle))
        p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)


def navigate_mobile(carPos, carOrn, sensor_readings, targetPos):

    carEuler = p.getEulerFromQuaternion(carOrn)
    carYaw = carEuler[2]
    hitTo_angle = []
    hitTo_Fraction = []

    angle_from_target_to_car = np.arctan2((targetPos[1] - carPos[1]), (targetPos[0] - carPos[0])) - carYaw
    if angle_from_target_to_car < -math.pi:
        angle_from_target_to_car = angle_from_target_to_car + 2 * math.pi
    if angle_from_target_to_car > math.pi:
        angle_from_target_to_car = angle_from_target_to_car - 2 * math.pi

    
    steering_angle = 0.0

    for i, sensor_reading in enumerate(sensor_readings):
        hitFraction = sensor_reading[2]

        localHitTo = [rayFrom[i][0] + hitFraction * (rayTo[i][0] - rayFrom[i][0]),
                      rayFrom[i][1] + hitFraction * (rayTo[i][1] - rayFrom[i][1]),
                      rayFrom[i][2] + hitFraction * (rayTo[i][2] - rayFrom[i][2])]

        rayangle_in_car_1 = np.arctan2((localHitTo[1] - rayFrom[i][1]), (localHitTo[0] - rayFrom[i][0]))

        hitTo_angle.append(rayangle_in_car_1)
        hitTo_Fraction.append(hitFraction)

    min_angle_difference = 2 * math.pi
    min_rayIndex = np.round(numRays / 2)
    for i in range(0, len(hitTo_Fraction)):
        if hitTo_Fraction[i] > 0.7:
            angle_difference = abs(hitTo_angle[i] - angle_from_target_to_car)
            if angle_difference < min_angle_difference:
                min_angle_difference = angle_difference
                min_rayIndex = i

    min_distance_to_obstacle = 0.12  
    is_obstacle_nearby = any(sensor_reading[2] < min_distance_to_obstacle for sensor_reading in sensor_readings)

    if is_obstacle_nearby:
        
        closest_obstacle_index = np.argmin([sensor_reading[2] for sensor_reading in sensor_readings])
        obstacle_angle = hitTo_angle[closest_obstacle_index]
       
        if obstacle_angle > 0:
            steering_angle = -np.radians(19)  
        elif obstacle_angle < 0:
            steering_angle = np.radians(19)  
        return steering_angle

    return angle_from_target_to_car



while (True):
    nowTime = time.time()
    nowControlTime = time.time()
    nowLidarTime = time.time()
    if (nowLidarTime - lastLidarTime > .03):

        carPos, carOrn = p.getBasePositionAndOrientation(car)


        numThreads = 0
        results = p.rayTestBatch(rayFrom, rayTo, numThreads, parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
        final_goal_pos = [10.99, 10.99]
        target_ori = [0, 0]
        carPos, carOrn = p.getBasePositionAndOrientation(car)


        maxForce = 21.579
        targetVelocity = 8.947
        steeringAngle = -0.432
        turn_angle = 0

        for i in range(numRays):
            hitObjectUid = results[i][0]
            hitFraction = results[i][2]
            hitPosition = results[i][3]
            if (hitFraction == 1.):
                p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=rayIds[i],
                                   parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
            else:
                localHitTo = [rayFrom[i][0] + hitFraction * (rayTo[i][0] - rayFrom[i][0]),
                              rayFrom[i][1] + hitFraction * (rayTo[i][1] - rayFrom[i][1]),
                              rayFrom[i][2] + hitFraction * (rayTo[i][2] - rayFrom[i][2])]
                p.addUserDebugLine(rayFrom[i], localHitTo, rayHitColor, replaceItemUniqueId=rayIds[i],
                                   parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
        lastLidarTime = nowLidarTime
        test_angle = navigate_mobile(carPos,carOrn,results,final_goal_pos)
        drive_the_mobile(carPos,carOrn,test_angle,final_goal_pos)
        
        if (useRealTimeSim == 0):
            frame += 1
            if(frame==1):
                time.sleep(0.95)
            p.stepSimulation()
        lastControlTime = nowControlTime
