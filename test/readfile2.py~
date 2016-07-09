from OCC.Display.SimpleGui import init_display
from OCC.TopoDS import (TopoDS_Shape, topods_Vertex)
from OCC.StlAPI import StlAPI_Reader

from OCC.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut
from OCC.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_Transform, BRepBuilderAPI_RoundCorner
from OCC.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeWedge, BRepPrimAPI_MakeSphere, BRepPrimAPI_MakeTorus
from OCC.gp import gp_Vec, gp_Ax2, gp_Pnt, gp_Dir, gp_Pln, gp_Trsf, gp_Circ, gp
from OCC.BRep import BRep_Tool

from OCCUtils import Topo, Common, edge
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

import yaml

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
#    print("%.3fs necessary to perform slice." % total_time)
    return sections

def get_connected_face_hash(whole_shape, part_shape, hash_faces):
    #whole_shape is the whole shape, can have multiple separate faces
    #return a hash of all faces connected to part_shape, whether directly or indirectly
    #the value of hash is 1, have to loop through Topo(whole_shape).faces to get to the faces
    topo_w = Topo(whole_shape)
    topo_p = Topo(part_shape)
    new_faces = {}
    for v in topo_p.vertices():
        for f in topo_w.faces_from_vertex(v):
            if not f.__hash__() in hash_faces:
                hash_faces[f.__hash__()] = 1
                new_faces[f.__hash__()]=f
    for k, v in new_faces.items():
        get_connected_face_hash(whole_shape, v, hash_faces)

def get_connected_edge_hash(whole_shape, part_shape, hash_edges):
    #whole_shape is the whole shape, can have multiple separate edges
    #return a hash of all edges connected to part_shape, whether directly or indirectly
    #the value of hash is 1, have to loop through Topo(whole_shape).edges to get to the edges
    topo_w = Topo(whole_shape)
    topo_p = Topo(part_shape)
    new_edges = {}
    #loop through vertices in part
    #   loop through edges connected to a vertex in whole
    #       add the edge to hash_edges
    #loop through all vertices of new edges
    for v in topo_p.vertices():
        for e in topo_w.edges_from_vertex(v):
            if not e.__hash__() in hash_edges:
                hash_edges[e.__hash__()] = 1
                new_edges[e.__hash__()]=e
    for k, v in new_edges.items():
        get_connected_edge_hash(whole_shape, v, hash_edges)

def get_nearest(shape1, shape2):
    distance = BRepExtrema_DistShapeShape()
    distance.LoadS1(shape1)
    distance.LoadS2(shape2)
    distance.Perform()
    while not distance.IsDone():
        print("waiting for get_nearest")
        time.sleep(0.01)
    return distance

def get_front_surface(stl_file, p1, p2):
    stl_reader = StlAPI_Reader()
    stl_box = TopoDS_Shape()
    stl_reader.Read(stl_box, stl_file)
    box = BRepPrimAPI_MakeBox(p1, p2).Shape()
    CommonSurface = BRepAlgoAPI_Common(box, stl_box).Shape()
    topo = Topo(CommonSurface)

    #ais_stl_box = display.DisplayShape(stl_box)
    #ais_box = display.DisplayShape(box)
    #display.Context.SetTransparency(ais_box, 0.8)
    #display.Context.SetTransparency(ais_stl_box, 0.5)

    distance = BRepExtrema_DistShapeShape()
    orig = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeVertex(gp_Pnt(0,0,0))
    distance.LoadS1(orig.Shape())
    min_distance = 999999999
    face_near = None
    for face in topo.faces():
        distance.LoadS2(face)
        distance.Perform()
        while not distance.IsDone():
            print("waiting for distance")
            time.sleep(0.01)
        d = distance.Value()
        if d<min_distance:
            min_distance = d
            face_near = face
    #face_near is nearest from the origin, remove all faces not connected to it
    hash_faces = {}
    get_connected_face_hash(CommonSurface, face_near, hash_faces)
    for f in topo.faces():
        if f.__hash__() not in hash_faces:
            CommonSurface = BRepAlgoAPI_Cut(CommonSurface, f).Shape()
    return CommonSurface

def get_wires_from_section(section):
#    print("get_wires_from_section 001")
#    OCCUtils.Topology.dumpTopology(section.Shape())
    #result of an intersection between two shapes can be multiple discontinued wires, find each of them and return a list of wires
    topo_s = Topo(section.Shape())
    wires = []
    for e in topo_s.edges():
        found_edge_in_wire = False
        e_hash = e.__hash__()
        for w in wires:
            if e_hash in w:
                found_edge_in_wire = True
        if not found_edge_in_wire:
            new_wire = {}
            get_connected_edge_hash(section.Shape(), e, new_wire)
            wires.append(new_wire)

    for e in topo_s.edges():
        e_hash = e.__hash__()
        for w in wires:
            if e_hash in w:
                w[e_hash] = e
    occ_wires = []
    for w in wires:
        occ_seq = TopTools_ListOfShape()
        wire_make = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire()
        for h, e in w.items():
            occ_seq.Append(e)
        wire_make.Add(occ_seq)
        occ_wires.append(wire_make.Wire())
    for w in occ_wires:
        e_l = get_edges_length(w)
#        print("wire length %.3fs " % e_l)
#        OCCUtils.Topology.dumpTopology(w)

    return occ_wires
    #time.sleep(10)


#create sections along Z axis
#find the longest intersection line
#sweep along this long
#find the pipe's intersection line on the surface
#sweep along this line

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

def getStripBoundary(aShape, spine, strip_width):
#    print("getStripBoundary 0000*********")
#    OCCUtils.Topology.dumpTopology(spine)
    brt = OCC.BRep.BRep_Tool()
    pnt = brt.Pnt(Topo(spine).ordered_vertices_from_wire(spine).next())
    circle = gp_Circ(gp_Ax2(pnt, OCC.gp.gp_DZ()), strip_width)
    profile_edge = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire(OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle).Edge())
    pipe = BRepOffsetAPI_MakePipeShell(spine)
    pipe.Add(profile_edge.Shape(), False, True)
    pipe.SetTransitionMode(BRepBuilderAPI_RoundCorner)
    pipe.Build()
    while not pipe.IsDone():
        print("waiting for pipe")
        time.sleep(0.01)
#    print("getStripBoundary 0020*********")
#    OCCUtils.Topology.dumpTopology(pipe.Shape())
    section = OCC.BRepAlgoAPI.BRepAlgoAPI_Section(pipe.Shape(), aShape)
    section.Build()
    while not section.IsDone():
        print("waiting for section")
        time.sleep(0.01)
#        print("getStripBoundary 0030*********")
#        OCCUtils.Topology.dumpTopology(section.Shape())
    return section

def sweep_face(aFace, initial_section, sweep_width):
    sweep_wires = []
    section = initial_section
    initial_z = 999999999
    last_z = -initial_z
    while section:
        section_wires = get_wires_from_section(section)
        #a section might have 1 or 2 wires, only move up
        section = None
        for w in section_wires:
            wire_bbox = Bnd_Box()
            OCC.BRepBndLib.brepbndlib_Add(w, wire_bbox)
            xmin, ymin, zmin, xmax, ymax, zmax = wire_bbox.Get()
            if zmin > last_z+0.01:
                last_z = zmin
                sweep_wires.append(w)
#               print("sweep_face: 0033")
#               OCCUtils.Topology.dumpTopology(valid_edges[0])
                section2 = getStripBoundary(front_face, w, sweep_width)
                if section2:
#                    print("sweep_face: 0043")
#                    OCCUtils.Topology.dumpTopology(section2.Shape())
                     section=section2
    return sweep_wires

def get_face_normal(face):
#    print("get_face_normal 0010")
#    OCCUtils.Topology.dumpTopology(face)
    bf = BRepGProp_Face(face)
    bounds = bf.Bounds()
    vec = gp_Vec()
    zDir = gp().DZ()
    pt = gp_Pnt()
    #get a normal vector to the face
    bf.Normal(bounds[0],bounds[1],pt,vec)
    return gp_Dir(vec)

def get_vertex_normal(vertex, shape):
    #a vertex belongs to multiple faces on the shape
    #can only choose one face when calculating vertex normal
    #preference tells which face is preferred (preferred direction, x+,y-,z+)
    distance = get_nearest(vertex, shape)
    #always use the first solution
#    print("get_vertex_normal 0010")
#    print(distance.Value())
#    print(distance.NbSolution())
    #SupportOnShape1, 2 is 1 based, not 0 based
    nearest_shape = distance.SupportOnShape2(1)
    topo_nearest_shape = Topo(nearest_shape)
    topo_shape = Topo(shape)
    face = None
#    print("get_vertex_normal 0020")
#    OCCUtils.Topology.dumpTopology(nearest_shape)
#    print(TopAbs_VERTEX)
#    print(nearest_shape.ShapeType())
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

def get_ordered_vertices_from_wire(wire):
    vertices = []
    vertex_hash = {}
    wire_topo=Topo(wire)
    for v in wire_topo.ordered_vertices_from_wire(wire):
        #the last vertex will be missing from ordered_vertices_from_wire
        vertices.append(v)
        vertex_hash[v.__hash__()]=1
    for v in wire_topo.vertices():
        #add the missing vertex
        if v.__hash__() not in vertex_hash:
            vertices.append(v)
            vertex_hash[v.__hash__()]=1
    return vertices

def format_wire_for_roboDK(wire, is_reverse=False):
    vertices = get_ordered_vertices_from_wire(wire)
    brt = BRep_Tool()
    wire=[]
    for i in range(len(vertices)):
        index = i
        if is_reverse:
            index = len(vertices)-1-i
        v=vertices[index]
        pnt = brt.Pnt(topods_Vertex(v))
        normal = get_vertex_normal(v, front_face)
        #direction for tool is towards face, which is reverse of the face's normal
        wire.append({"location": [pnt.X(), pnt.Y(), pnt.Z()], "direction": [-normal.X(), -normal.Y(), -normal.Z()]})
    return wire


#reduce number of vertices on a wire, to smoothen robotic arm's movement
def reduce_wire_edge(wire, max_variance):
    #a vertex can be removed if it's sufficiently close to the line between the two vertices on its side
    #start from one end, them move to the other end
    #print("reduce_wire_edge 0010")
    wire_keep = []
    old_wire_topo = Topo(wire)
    #OCCUtils.Topology.dumpTopology(wire)
    vertices = get_ordered_vertices_from_wire(wire)
    #pprint(vertices)
    edge_start = 0
    wire_keep.append(edge_start)
    for i in range(edge_start+2, len(vertices)):
        new_edge = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(vertices[edge_start], vertices[i]).Edge()
        for j in range(edge_start+1, i):
            distance = get_nearest(vertices[j], new_edge)
            if distance.Value()>max_variance:
                #the vertices[i] cannot be skipped
                wire_keep.append(j)
                edge_start = j
                break
    #add the last vertex
    wire_keep.append(len(vertices)-1)
    #build new wire from these vertises
    new_wire = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire()
    occ_seq = TopTools_ListOfShape()
    for i in range(len(wire_keep)-1):
        new_edge = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(vertices[wire_keep[i]], vertices[wire_keep[i+1]]).Edge()
        occ_seq.Append(new_edge)
    new_wire.Add(occ_seq)
    return new_wire.Wire()


display, start_display, add_menu, add_function_to_menu = init_display()

max_vertex_variance = 0.01
sweep_width=50
front_face = get_front_surface('./models/cylindar.stl', gp_Pnt(100.0, 0.0, -500), gp_Pnt(1100.0, 4000.0, 500.0))
long_slice = get_lowest_long_slice(front_face, 20)
#OCCUtils.Topology.dumpTopology(long_slice.Shape())
sweep_wires = sweep_face(front_face, long_slice, sweep_width)

wires_roboDK = {'wires': []}
#display.DisplayShape(front_face)
for idx, w in enumerate(sweep_wires):
    #smoothen wire, remove unneeded edges from wire
    #reverse direction for odd numbered wires
    wire_reduced = reduce_wire_edge(w, max_vertex_variance)
    wire_roboDK = format_wire_for_roboDK(wire_reduced, (idx % 2)==0)
    wires_roboDK['wires'].append(wire_roboDK)
    display.DisplayShape(wire_reduced)
display.FitAll()
start_display()

outstream = open('./curves/cylindar.yml', 'w')
yaml.dump(wires_roboDK, outstream, default_flow_style=False)
outstream.close()

with open('./curves/cylindar.csv', 'w') as csv_outstream:
    for w in wires_roboDK['wires']:
        for v in w:
            location = v['location']
            direction = v['direction']
            csv_outstream.write("%f, %f, %f, %f, %f, %f\n" % (location[0], location[1], location[2], direction[0], direction[1], direction[2]))



