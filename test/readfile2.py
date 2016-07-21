from OCC.Display.SimpleGui import init_display
from OCC.TopoDS import (TopoDS_Shape, topods_Vertex)
from OCC.StlAPI import StlAPI_Reader

from OCC.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut
from OCC.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_Transform, BRepBuilderAPI_RoundCorner, BRepBuilderAPI_RightCorner, BRepBuilderAPI_Transformed
from OCC.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeWedge, BRepPrimAPI_MakeSphere, BRepPrimAPI_MakeTorus
from OCC.gp import gp_Vec, gp_Ax2, gp_Pnt, gp_Dir, gp_Pln, gp_Trsf, gp_Circ, gp
from OCC.BRep import BRep_Tool

from OCCUtils import Topo, Common, edge
from OCCUtils.Topology import WireExplorer
from OCC.Bnd import Bnd_Box
import OCC
import OCCUtils
import time
from OCC.BRepOffsetAPI import BRepOffsetAPI_MakePipe, BRepOffsetAPI_MakePipeShell
from OCC.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.TopTools import TopTools_ListOfShape
from pprint import pprint
from OCC.BRepGProp import BRepGProp_Face
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

sys.path.append('../')
import sweeper.utilities as ut

def show_log(function, message):
    if function in logging_functions:
        print("%s  %s: %s" % (function, datetime.datetime.now(), message))

def slicer(shape, deltaZ):
    # Param
    bbox = Bnd_Box()
    OCC.BRepBndLib.brepbndlib_Add(shape, bbox)
    xmin, ymin, Zmin, xmax, ymax, Zmax = bbox.Get()
    D = gp_Dir(0., 0., 1.)  # the z direction
    # Perform slice
    sections = []
    init_time = time.time()  # for total time computation
    iMax = int((Zmax-Zmin)/deltaZ)
    for i in range(iMax):
        # Create Plane defined by a point and the perpendicular direction
        z = Zmin+i*deltaZ
        P = gp_Pnt(0, 0, z)
        Pln = gp_Pln(P, D)
        face = BRepBuilderAPI_MakeFace(Pln).Shape()
        # Computes Shape/Plane intersection
        section = OCC.BRepAlgoAPI.BRepAlgoAPI_Section(shape, face)
        if section.IsDone():
            sections.append(section)
    total_time = time.time() - init_time
#    show_log("slicer", "%.3fs necessary to perform slice." % total_time)
    return sections

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

def get_edges_length(aShape):
    t_length = 0
    for e in Topo(aShape).edges():
        oe = OCCUtils.edge.Edge(e)
        t_length += oe.length()
    return t_length

def get_lowest_long_slice(aShape, zDelta):
    #create sections along Z axis
    slices = slicer(aShape, zDelta)
    max_slice_length=-1
    max_slice = None
    for s in slices:
        #find the longest intersection line with the smallest z value
        #slices start from small z
        slice_length=get_edges_length(s.Shape())
        if slice_length > max_slice_length+0.01:
            max_slice_length = slice_length
            max_slice = s
    return max_slice

def get_face_normal(face):
    bf = BRepGProp_Face(face)
    bounds = bf.Bounds()
    vec = gp_Vec()
    zDir = gp().DZ()
    pt = gp_Pnt()
    #get a normal vector to the face
    bf.Normal(bounds[0],bounds[1],pt,vec)
    #get cos between normal and x axis [1,0,0]
    direction = gp_Dir(vec)
    return direction

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
    n = get_face_normal(face)
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
        #direction for tool is towards face, which is reverse of the face's normal
        wire.append({"location": [pnt.X(), pnt.Y(), pnt.Z()], "direction": [-normal.X(), -normal.Y(), -normal.Z()], "path_direction": [path_direction.item(0),path_direction.item(1),path_direction.item(2)]})
    return wire
    
display, start_display, add_menu, add_function_to_menu = init_display()

logging_functions = ["get_wires_from_section", "merge_nearby_edges", "getStripBoundary", "sweep_face", "reduce_wire_edge"]

object_name='full-cylindar-sphere-top-from-rhino'
blocks = [
          {'blockid': '01', 'sweep_orientation': 'x', 'view_port_bottom_left':[-4000.0,-1200.0,-400.0], 'view_port_top_right': [4000.0,-400.0,400.0]},
          {'blockid': '02', 'sweep_orientation': 'x', 'view_port_bottom_left':[-4000.0,-1200.0,400.0], 'view_port_top_right': [4000.0,-400.0,1200.0]},
          {'blockid': '03', 'sweep_orientation': 'y', 'view_port_bottom_left':[-4000.0,-400.0,-400.0], 'view_port_top_right': [0.0,400.0,400.0]},
          {'blockid': '04', 'sweep_orientation': 'y', 'view_port_bottom_left':[-4000.0,-400.0,400.0], 'view_port_top_right': [0.0,400.0,1200.0]}
          ]
for b in blocks:
    print("blockid: %s" % b['blockid'])
    sweeper=ut.surface_sweeper({'sweep_width': 30.0, 'max_vertex_variance': 0.001,
                            'max_sweep_line_variance': 1.0, 'sweep_orientation': b['sweep_orientation'], 'wire_join_max_distance': 45,
                            'path_extension_distance': 10.0})
    vp_bf=b['view_port_bottom_left']
    vp_tr=b['view_port_top_right']
    front_face = get_front_surface('../freeCAD/'+object_name+'.stl', gp_Pnt(vp_bf[0], vp_bf[1], vp_bf[2]), gp_Pnt(vp_tr[0], vp_tr[1], vp_tr[2]))
    long_slice = get_lowest_long_slice(front_face, 20)
    sweep_wires_upside = sweeper.sweep_face(front_face, long_slice, 'up')
    sweep_wires_downside = sweeper.sweep_face(front_face, long_slice, 'down')
    del sweep_wires_downside[0]
    sweep_wires = sweep_wires_upside + sweep_wires_downside

    wires_roboDK = {'wires': []}
    ais_front = display.DisplayShape(front_face)
    display.Context.SetTransparency(ais_front, 0.8)

    for idx, w in enumerate(sweep_wires):
        #smoothen wire, remove unneeded edges from wire
        #reverse direction for odd numbered wires
        #display.DisplayShape(w)
        wire_roboDK = format_wire_for_roboDK(w, (idx % 2)==0)
        wires_roboDK['wires'].append(wire_roboDK)
        display.DisplayShape(w)
    
    outstream = open('./curves/'+object_name+b['blockid']+'.yml', 'w')
    yaml.dump(wires_roboDK, outstream, default_flow_style=False)
    outstream.close()
    
    with open('./curves/'+object_name+b['blockid']+'.csv', 'w') as csv_outstream:
        for w in wires_roboDK['wires']:
            for v in w:
                location = v['location']
                direction = v['direction']
                csv_outstream.write("%f, %f, %f, %f, %f, %f\n" % (location[0], location[1], location[2], direction[0], direction[1], direction[2]))

display.FitAll()
start_display()
