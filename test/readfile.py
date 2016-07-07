from OCC.Display.SimpleGui import init_display
from OCC.TopoDS import TopoDS_Shape
from OCC.StlAPI import StlAPI_Reader

from OCC.BRepAlgoAPI import BRepAlgoAPI_Common
from OCC.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_Transform
from OCC.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeWedge, BRepPrimAPI_MakeSphere, BRepPrimAPI_MakeTorus
from OCC.gp import gp_Vec, gp_Ax2, gp_Pnt, gp_Dir, gp_Pln, gp_Trsf, gp_Circ
from OCC.BRep import BRep_Tool

from OCCUtils import Topo
from OCC.Bnd import Bnd_Box
import OCC
import OCCUtils
import time
from OCC.BRepOffsetAPI import BRepOffsetAPI_MakePipe

from OCCUtils.Construct import make_closed_polygon
p1 = gp_Pnt(0, 0, 0)
p2 = gp_Pnt(0, 10, 0)
p3 = gp_Pnt(10, 10, 0)
p4 = gp_Pnt(10, 0, 0)
rect = make_closed_polygon(p1,p2,p3,p3)

display, start_display, add_menu, add_function_to_menu = init_display()
#my_box = BRepPrimAPI_MakeBox(10., 20., 30.).Shape()

stl_reader = StlAPI_Reader()
stl_box = TopoDS_Shape()
stl_reader.Read(stl_box, './models/box.stl')

axe = gp_Ax2(gp_Pnt(-10, 1, 1), gp_Dir(0, 0, 1))
box = BRepPrimAPI_MakeBox(axe, 50, 15, 15).Shape()
CommonSurface = BRepAlgoAPI_Common(box, stl_box).Shape()

topo = Topo(CommonSurface)
display.EraseAll()
x_mid_max = -100
front_face = None
for face in topo.faces():
   bbox = Bnd_Box()
   OCC.BRepBndLib.brepbndlib_Add(face, bbox)
   xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
   x_mid = (xmin+xmax)/2
   if x_mid>x_mid_max:
      x_mid_max = x_mid
      front_face = face

t_face = Topo(front_face)
OCCUtils.Topology.dumpTopology(front_face)
wires = t_face.wires()
z_center_min_edge=1000
lowest_edge = None
lowest_wire = None
for w in wires:
   #display.DisplayShape(w)
   edges = Topo(w).edges()
   for e in edges:
      bbox = Bnd_Box()
      OCC.BRepBndLib.brepbndlib_Add(e, bbox)
      xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
      #find the lowest edge, the edge with the smallest zmin+zmax
      if zmin+zmax<z_center_min_edge:
         z_center_min_edge=zmin+zmax
         lowest_edge = e
         lowest_wire = w
         #display.DisplayShape(e)
print z_center_min_edge
display.DisplayShape(lowest_edge)
vertices = Topo(lowest_edge).vertices()

wire_make = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire()
wire_make.Add(lowest_edge)
print wire_make
OCCUtils.Topology.dumpTopology(wire_make.Wire())

# pipe
brt = OCC.BRep.BRep_Tool()
pnt = brt.Pnt(vertices.next())

import pprint
pp = pprint.PrettyPrinter(indent=4)
print("orientation")
curve, f1, l1 = brt.Curve(lowest_edge)
print(f1)
print(l1)
pp.pprint(curve.GetObject())
print("done")
dotp=gp_Pnt()
curve.GetObject().D0(0, dotp)
aVec = gp_Vec(dotp.X(), dotp.Y(), dotp.Z())
aDir = gp_Dir(aVec)
circle = gp_Circ(gp_Ax2(pnt, aDir),1)
profile_edge = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle).Edge()
pipe = BRepOffsetAPI_MakePipe(wire_make.Wire(), profile_edge).Shape()
section = OCC.BRepAlgoAPI.BRepAlgoAPI_Section(pipe, front_face)
section.Build()
if section.IsDone():
   display.DisplayShape(section.Shape(), update=True)
section.Destroy()
display.DisplayShape(pipe, update=True)
#circle1 = gp_Circ(gp_Ax2(gp_Pnt(0,50,0),gp().DX()),5)
#profile_edge = BRepBuilderAPI_MakeEdge(circle1).Edge()
#for v in vertices:
   #print(pnt)
   #display.DisplayShape(v)
   #print(v)

display.FitAll()

#display.EraseAll()
#ais_stl_box = display.DisplayShape(stl_box)
#ais_box = display.DisplayShape(box)
#display.Context.SetTransparency(ais_box, 1.0)
#display.Context.SetTransparency(ais_stl_box, 0.5)
#display.DisplayShape(CommonSurface)
#display.FitAll()

start_display()

