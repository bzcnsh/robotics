from OCC.Display.SimpleGui import init_display
from OCC.TopoDS import (TopoDS_Shape, topods_Vertex)
from OCC.StlAPI import StlAPI_Reader

from OCC.BRepAlgoAPI import BRepAlgoAPI_Common
from OCC.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.gp import gp_Pnt
from OCC.BRep import BRep_Tool

from OCCUtils import Topo, Common, edge
import OCC
import OCCUtils
import time
from pprint import pprint
from OCC.TopAbs import (TopAbs_VERTEX, TopAbs_EDGE, TopAbs_FACE, TopAbs_WIRE,
                        TopAbs_SHELL, TopAbs_SOLID, TopAbs_COMPOUND,
                        TopAbs_COMPSOLID)
import OCC.RWStl

import yaml
import datetime
import pickle
import numpy
import scipy.linalg

import sys
import getopt

sys.path.append('../')
import sweeper.utilities as ut

def read_yaml(filename):
   stream = open(filename, 'r')
   data = yaml.load(stream)
   stream.close()
   return data

def show_log(function, message):
    if function in logging_functions:
        print("%s  %s: %s" % (function, datetime.datetime.now(), message))

def get_front_surface(stl_file, p1, p2):
    show_log("get_front_surface", "001")

    stl_reader = StlAPI_Reader()
    stl_shape = TopoDS_Shape()
    show_log("get_front_surface", "005")
#    Read takes 5 minutes
    stl_reader.Read(stl_shape, stl_file)
    #pickle.dump(stl_shape, open( "get_front_surface 001.tmp", "wb" ) )
#    stl_shape = pickle.load( open( "./get_front_surface 001.tmp", "rb" ) )
#    t = Topo(stl_shape)
#    print("number of faces: %i" % (t.number_of_faces()))
    #OCC.RWStl.rwstl().ReadFile(stl_file)
    #time.sleep(10)
    #cos theta should be less than 0.5, theta is the angle between 1,0,0 and the face normal
    show_log("get_front_surface", "010")
    box = BRepPrimAPI_MakeBox(p1, p2).Shape()
    show_log("get_front_surface", "012")
#   BRepAlgoAPI_Common takes 3 minutes
    CommonSurface = BRepAlgoAPI_Common(box, stl_shape).Shape()
    ais_stl_shape = display.DisplayShape(stl_shape)
#    ais_box = display.DisplayShape(box)
#    ais_common = display.DisplayShape(CommonSurface)
#    display.Context.SetTransparency(ais_box, 0.8)
    display.Context.SetTransparency(ais_stl_shape, 0.8)
#    orig = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeVertex(gp_Pnt(0,0,0))
#    display.DisplayShape(orig.Shape())

    return CommonSurface

def get_longest_slice(aShape, delta):
    #create sections along Z axis
    slices = sweeper.get_slices(aShape, delta)
    max_slice_length=-1
    max_slice = None
    for s in slices:
        #find the longest intersection line with the smallest z value
        #slices start from small z
        slice_length=sweeper.get_edges_length(s.Shape())
        if slice_length > max_slice_length+0.01:
            max_slice_length = slice_length
            max_slice = s
    return max_slice

def get_vertex_normal(vertex, shape):
    #a vertex belongs to multiple faces on the shape
    #can only choose one face when calculating vertex normal
    #preference tells which face is preferred (preferred direction, x+,y-,z+)
    distance = sweeper.get_nearest(vertex, shape)
    #always use the first solution
    show_log("get_vertex_normal", "0010")
    #SupportOnShape1, 2 is 1 based, not 0 based
    nearest_shape = distance.SupportOnShape2(1)
    topo_nearest_shape = Topo(nearest_shape)
    topo_shape = Topo(shape)
    face = None
    show_log("get_vertex_normal", "0020")
    if nearest_shape.ShapeType()==TopAbs_VERTEX:
        face=topo_shape.faces_from_vertex(nearest_shape).next()
    if nearest_shape.ShapeType()==TopAbs_EDGE:
        face=topo_shape.faces_from_edge(nearest_shape).next()
    if nearest_shape.ShapeType()==TopAbs_WIRE:
        face=topo_shape.faces_from_wire(nearest_shape).next()
    if nearest_shape.ShapeType()==TopAbs_FACE:
        wire=topo_shape.wires_from_face(nearest_shape).next()
        face=topo_shape.faces_from_wire(wire).next()
    assert face, "unhandled nearest_shape type"
    n = sweeper.get_face_normal(face)
    return n

def format_wire_for_roboDK(wire, is_reverse=False):
    vertices = sweeper.get_ordered_vertices_from_wire(wire)
    brt = BRep_Tool()
    wire=[]
    last_path_direction = None
    for i in range(len(vertices)):
        index = i
        next_index = index + 1
        if is_reverse:
            index = len(vertices)-1-i
            next_index = index - 1
        v=vertices[index]
        pnt = brt.Pnt(topods_Vertex(v))
        normal = get_vertex_normal(v, front_face)
        if not ((index==0 and is_reverse) or (index==len(vertices)-1 and not is_reverse)):
            pnt_next = brt.Pnt(topods_Vertex(vertices[next_index]))
            mat_pnt = numpy.mat([pnt.X(), pnt.Y(), pnt.Z()])
            mat_pnt_next = numpy.mat([pnt_next.X(), pnt_next.Y(), pnt_next.Z()])
            path_direction = mat_pnt_next-mat_pnt
            path_direction = path_direction/scipy.linalg.norm(path_direction)
            last_path_direction = path_direction
        else:
            path_direction = last_path_direction
        #direction should be away from base_position
        p1 = [pnt.X()+normal.X(), pnt.Y()+normal.Y(), pnt.Z()+normal.Z()]
        p2 = [pnt.X()-normal.X(), pnt.Y()-normal.Y(), pnt.Z()-normal.Z()]
        #normal vector should point towards base_position
        if sweeper.get_distance_points(p1, sweeper.base_position) < sweeper.get_distance_points(p2, sweeper.base_position):
            direction = [normal.X(), normal.Y(), normal.Z()]
        else:
            direction = [-normal.X(), -normal.Y(), -normal.Z()]
        wire.append({"location": [pnt.X(), pnt.Y(), pnt.Z()], "direction": direction, "path_direction": [path_direction.item(0),path_direction.item(1),path_direction.item(2)]})
    return wire
    
display, start_display, add_menu, add_function_to_menu = init_display()

logging_functions = ["get_wires_from_section", "merge_nearby_edges", "getStripBoundary", "sweep_face", "reduce_wire_edge"]
#object_name='full-cylindar-sphere-top-from-rhino'

try:
    opts, args = getopt.getopt(sys.argv[1:], 'j:')
except getopt.GetoptError:
      print 'missing job file from command line'
      sys.exit(2)
#job = read_yaml("jobs/full-cylindar-sphere-top-from-rhino.yml")
for opt, arg in opts:
      if opt == '-j':
         job_file = arg
job = read_yaml(job_file)

#work item is in bound by box [-1000, -1000, -1000[, [1000, 2000, 1000]
blocks = job['blocks'];

for b in blocks:
    print("blockid: %s" % b['blockid'])
    settings = job['job_settings']
    for i in b.keys():
        settings[i] = b[i]
        sweeper=ut.surface_sweeper(settings)
    vp_bf=b['view_port_bottom_left']
    vp_tr=b['view_port_top_right']
    front_face = get_front_surface(job['object_file'], gp_Pnt(vp_bf[0], vp_bf[1], vp_bf[2]), gp_Pnt(vp_tr[0], vp_tr[1], vp_tr[2]))
    long_slice = get_longest_slice(front_face, sweeper.sweep_width)
    sweep_wires_downside = sweeper.sweep_face(front_face, long_slice, 'down')
    sweep_wires_upside = sweeper.sweep_face(front_face, long_slice, 'up')
    del sweep_wires_downside[0]
    sweep_wires = sweep_wires_upside + sweep_wires_downside
    xmin, ymin, Zmin, xmax, ymax, Zmax = sweeper.get_shape_boundary(sweep_wires)
    wires_roboDK = {'wires': [], 'boundary': [[xmin, ymin, Zmin], [xmax, ymax, Zmax]], 'base_position':b['base_position']}
    ais_front = display.DisplayShape(front_face)
    display.Context.SetTransparency(ais_front, 0.8)

    for idx, w in enumerate(sweep_wires):
        #smoothen wire, remove unneeded edges from wire
        #reverse direction for odd numbered wires
        #display.DisplayShape(w)
        wire_roboDK = format_wire_for_roboDK(w, (idx % 2)==0)
        wires_roboDK['wires'].append(wire_roboDK)
        
        display.DisplayShape(w)
    
    outstream = open(job['output_dir'] + '/' + job['object_name']+b['blockid']+'.yml', 'w')
    yaml.dump(wires_roboDK, outstream, default_flow_style=False)
    outstream.close()
    
    with open(job['output_dir'] + '/' + job['object_name'] + b['blockid']+'.csv', 'w') as csv_outstream:
        for w in wires_roboDK['wires']:
            for v in w:
                location = v['location']
                direction = v['direction']
                csv_outstream.write("%f, %f, %f, %f, %f, %f\n" % (location[0], location[1], location[2], direction[0], direction[1], direction[2]))

display.FitAll()
start_display()
