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

def get_front_surface(stl_file, p1, p2):
    stl_reader = StlAPI_Reader()
    stl_box = TopoDS_Shape()
    stl_reader.Read(stl_box, stl_file)
    box = BRepPrimAPI_MakeBox(p1, p2).Shape()
    CommonSurface = BRepAlgoAPI_Common(box, stl_box).Shape()
    topo = Topo(CommonSurface)
    x_min_min = 100
    #remove back surface, which is a rectange at a fixed X
    for face in topo.faces():
        bbox = Bnd_Box()
        OCC.BRepBndLib.brepbndlib_Add(face, bbox)
        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
        if xmin<=x_min_min:
            x_min_min=xmin
    for face in topo.faces():
        bbox = Bnd_Box()
        OCC.BRepBndLib.brepbndlib_Add(face, bbox)
        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
        if xmin==x_min_min:
            CommonSurface = BRepAlgoAPI_Cut(CommonSurface, face).Shape()
    return CommonSurface

#create sections along Z axis
#find the longest intersection line
#sweep along this long
#find the pipe's intersection line on the surface
#sweep along this line

def get_lowest_long_slice(aShape, zDelta):
    #create sections along Z axis
    slices = slicer(front_face, zDelta)
    max_slice_length=-1
    max_slice = None
    for s in slices:
        #find the longest intersection line with the smallest z value
        #slices start from small z
        OCCUtils.Topology.dumpTopology(s.Shape())
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
    last_z = -100
    while section:
        #a section might have multiple edges, only move up
        valid_edges = []
        min_valid_z = -100
        for e in Topo(section.Shape()).edges():
            edge_bbox = Bnd_Box()
            OCC.BRepBndLib.brepbndlib_Add(e, edge_bbox)
            edge_xmin, edge_ymin, edge_zmin, edge_xmax, edge_ymax, edge_zmax = edge_bbox.Get()
            if edge_zmin > last_z+0.01:
                valid_edges.append(e)
                if edge_zmin<min_valid_z or min_valid_z==-100:
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

display, start_display, add_menu, add_function_to_menu = init_display()
#my_box = BRepPrimAPI_MakeBox(10., 20., 30.).Shape()

#'./models/box.stl'
#view is along the X axis
stl_reader = StlAPI_Reader()
stl_box = TopoDS_Shape()
stl_reader.Read(stl_box, './models/cylindar.stl')


front_face = get_front_surface('./models/cylindar.stl', gp_Pnt(100.0, 0.0, -500), gp_Pnt(1100.0, 4000.0, 500.0))
long_slice = get_lowest_long_slice(front_face, 20)
sweep_wires = sweep_face(front_face, long_slice, 50)

OCCUtils.Topology.dumpTopology(front_face)
display.DisplayShape(front_face)

for l in sweep_wires:
    display.DisplayShape(l.Shape())

display.FitAll()
start_display()

