from OCC.Display.SimpleGui import init_display
from OCC.TopoDS import TopoDS_Shape
from OCC.StlAPI import StlAPI_Reader

from OCC.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut
from OCC.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_Transform, BRepBuilderAPI_RoundCorner
from OCC.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeWedge, BRepPrimAPI_MakeSphere, BRepPrimAPI_MakeTorus
from OCC.gp import gp_Vec, gp_Ax2, gp_Pnt, gp_Dir, gp_Pln, gp_Trsf, gp_Circ
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

display, start_display, add_menu, add_function_to_menu = init_display()

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
    print("%.3fs necessary to perform slice." % total_time)
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
        if not distance.IsDone():
            print("waiting for distance")
            time.sleep(1)
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
        occ_wires.append(wire_make)
        OCCUtils.Topology.dumpTopology(wire_make.Shape())
    #time.sleep(10)


#create sections along Z axis
#find the longest intersection line
#sweep along this long
#find the pipe's intersection line on the surface
#sweep along this line

def get_lowest_long_slice(aShape, zDelta):
    #create sections along Z axis
    slices = slicer(aShape, zDelta)
    max_slice_length=-1
    max_slice = None
    for s in slices:
        #find the longest intersection line with the smallest z value
        #slices start from small z
        slice_length=0
        for e in Topo(s.Shape()).edges():
            oe = OCCUtils.edge.Edge(e)
            slice_length += oe.length()
        if slice_length > max_slice_length+0.01:
            max_slice_length = slice_length
            max_slice = s
    return max_slice

def getStripBoundary(aShape, spine, strip_width):
    #print("0000000")
    #OCCUtils.Topology.dumpTopology(spine.Shape())
    brt = OCC.BRep.BRep_Tool()
    edges = Topo(spine.Shape()).edges()
    vertices = Topo(edges.next()).vertices()
    pnt = brt.Pnt(vertices.next())
    circle = gp_Circ(gp_Ax2(pnt, OCC.gp.gp_DZ()), strip_width)
    profile_edge = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire(OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle).Edge())
    pipe = BRepOffsetAPI_MakePipeShell(spine.Wire())
    if not pipe.IsDone():
        #print("waiting for pipe")
        time.sleep(0.05)
    pipe.Add(profile_edge.Shape(), False, True)
    pipe.SetTransitionMode(BRepBuilderAPI_RoundCorner)
    pipe.Build()
    section = OCC.BRepAlgoAPI.BRepAlgoAPI_Section(pipe.Shape(), aShape)
    section.Build()
    if section.IsDone():
        return section

def sweep_face(aFace, initial_section, sweep_width):
    sweep_wires = []
    section = initial_section
    initial_z = 999999999
    last_z = -initial_z
    while section:
        section_wires = get_wires_from_section(section)
        #a section might have multiple edges, only move up
        valid_edges = []
        min_valid_z = -initial_z
        for e in Topo(section.Shape()).edges():
            edge_bbox = Bnd_Box()
            OCC.BRepBndLib.brepbndlib_Add(e, edge_bbox)
            edge_xmin, edge_ymin, edge_zmin, edge_xmax, edge_ymax, edge_zmax = edge_bbox.Get()
            if edge_zmin > last_z+0.01:
                valid_edges.append(e)
                if edge_zmin<min_valid_z or min_valid_z==-initial_z:
                    min_valid_z = edge_zmin
        if valid_edges:
            occ_seq = TopTools_ListOfShape()
            for e in valid_edges:
                #if an edge does not share a vertex with any existing edge in the wire, Add won't work
                occ_seq.Append(e)
            wire_make = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire()
            wire_make.Add(occ_seq)
            sweep_wires.append(wire_make)
            section2 = getStripBoundary(front_face, wire_make, sweep_width)
            if section2:
                #OCCUtils.Topology.dumpTopology(section2.Shape())
                section=section2
                last_z = min_valid_z
            else:
                section=None
        else:
            section=None
    return sweep_wires

#my_box = BRepPrimAPI_MakeBox(10., 20., 30.).Shape()

#'./models/box.stl'
#view is along the X axis
stl_reader = StlAPI_Reader()
stl_box = TopoDS_Shape()
stl_reader.Read(stl_box, './models/cylindar.stl')


front_face = get_front_surface('./models/cylindar.stl', gp_Pnt(100.0, 0.0, -500), gp_Pnt(1100.0, 4000.0, 500.0))
long_slice = get_lowest_long_slice(front_face, 20)
OCCUtils.Topology.dumpTopology(long_slice.Shape())
sweep_wires = sweep_face(front_face, long_slice, 50)

#display.DisplayShape(front_face)
print(len(sweep_wires))
for l in sweep_wires:
    display.DisplayShape(l.Shape())

display.FitAll()
start_display()

