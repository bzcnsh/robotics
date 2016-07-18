# Type help("robolink") or help("robodk") for more information
# Press F5 to run the script
# Note: you do not need to keep a copy of this file, your python script is saved with the station
from robolink import *    # API to communicate with RoboDK
from robodk import *      # basic matrix operations
RL = Robolink()

# Notify user:
print('A Matlab version of this API is available in C:/RoboDK/Matlab/\n')
print('The API is also available in C#\n')
print('To edit this program:\nright click on the Python program, then, select "Edit Python script"\n')


import math
import numpy as np
from pprint import pprint
import numpy as np
import yaml
   
def vectors2matrix(A,B):
   #A starting vector
   #B ending vector
   #returns: 3x3 matrix to rotate A to B

   rcos = np.dot(A, B)
   C = np.cross(A, B)
   rsin = np.linalg.norm(C)
   C = C/rsin

   u=C[0]
   v=C[1]
   w=C[2]
   R=np.array([[rcos + u*u*(1-rcos),  -w * rsin + u*v*(1-rcos), v * rsin + u*w*(1-rcos)],
                      [ w * rsin + v*u*(1-rcos), rcos + v*v*(1-rcos), -u * rsin + v*w*(1-rcos)],
                      [-v * rsin + w*u*(1-rcos), u * rsin + w*v*(1-rcos), rcos + w*w*(1-rcos)]])

   return(R)

def point2matrix(point):
    matrix_opposite = np.array([[-1,  0,  0],
                     [ 0, -1,  0],
                     [ 0,  0, -1]])
    start_vector = np.array([0, 0, 1])
    direction = np.dot(np.array([point[3], point[4], point[5]]), matrix_opposite)
    rotation_matrix = vectors2matrix(start_vector, direction)
    four_by_four = np.zeros((4,4))
    four_by_four[:3, :3]=rotation_matrix

    four_by_four[0][3]=point[0]    
    four_by_four[1][3]=point[1]
    four_by_four[2][3]=point[2]
    four_by_four[3][3]=1
    return four_by_four


def matrix2pose(matrix):
    pose = Mat(4,4)
    for j in range(4):
        for i in range(4):
            pose[i,j] = matrix[i][j]
    return pose

def point2pose(point, matrix_more_change=None):
    m = point2matrix(point)
#    print("m:")
#    print(m)
    if matrix_more_change is None:
        p = matrix2pose(m)
    else:
        new_m = np.dot(m, matrix_more_change)
        p = matrix2pose(new_m)
#    print("p:")
#    print(p)
    return p                


def read_wires(filename):
   stream = open(filename, 'r')
   wires_roboDK = yaml.load(stream)
   stream.close()
   return wires_roboDK['wires']

def vertex_hash_to_array(h):
   return h['location']+h['direction']

def read_job_setting(filename):
   stream = open(filename, 'r')
   settings = yaml.load(stream)
   stream.close()
   return settings
   
wires = read_wires("C:\\Users\\Admin\\Documents\\VR_painting\\robotics\\test\\curves\\cylindar.yml")
settings = read_job_setting("C:\\Users\\Admin\\Documents\\roboDK\\cylindar.yml")

# Program example:
item = RL.Item('base')
if item.Valid():
    print('Item selected: ' + item.Name())
    print('Item posistion: ' + repr(item.Pose()))

#print('Items in the station:')
#itemlist = RL.ItemList()
#print(itemlist)

robot = RL.Item('UR10')
baseFrame = RL.Item('UR10 Base')
paintFrame = RL.Item('paintframe')

# get the home target and the welding targets:
robot.setFrame(paintFrame)
home = RL.Item('home')
#poseref = home.Pose()

#print('Home posistion: ' + repr(home.Pose()))
#robot.MoveJ(home)

distance_ready = np.array([[1.0, 0.0, 0.0, 0.0],[0.0, 1.0, 0.0, 0.0],[0.0, 0.0, 1.0, settings['wire_setting']['ready_offset']], [0.0, 0.0, 0.0, 1.0]])
distance_work = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],[0.0, 0.0, 1.0, settings['wire_setting']['work_offset']], [0.0, 0.0, 0.0, 1.0]])
distance_end = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, settings['wire_setting']['end_offset']], [0.0, 0.0, 0.0, 1.0]])
distance_job_end = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, settings['wire_setting']['job_end_offset']], [0.0, 0.0, 0.0, 1.0]])
for wire in wires:
    pprint(wire[0])
    wire_ready = point2pose(vertex_hash_to_array(wire[0]), distance_ready)
    wire_end = point2pose(vertex_hash_to_array(wire[-1]), distance_end)
    robot.MoveJ(wire_ready)
    for vertex in wire:
       vertex_pose = point2pose(vertex_hash_to_array(vertex), distance_work)
       robot.MoveL(vertex_pose)
    robot.MoveJ(wire_end)
job_end = point2pose(vertex_hash_to_array(wires[-1][-1]), distance_job_end)
robot.MoveJ(job_end)

#raise Exception('Program not edited.')


#for i in range(7):
#    ang = i*2*pi/6 #angle: 0, 60, 120, ...
#    #posei = poseref*rotz(ang)*transl(200,0,0)*rotz(-ang)
#    posei = poseref*rotz(ang)*transl(100,0,0)*rotz(-ang)
#    print(posei.__str__)
#    robot.MoveL(posei)
