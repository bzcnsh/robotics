from OCC.Display.SimpleGui import init_display
from OCC.TopoDS import TopoDS_Shape, topods, TopoDS_Compound
from OCC.StlAPI import StlAPI_Reader

from OCC.BRepAlgoAPI import BRepAlgoAPI_Common
from OCC.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_Transform
from OCC.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeWedge, BRepPrimAPI_MakeSphere, BRepPrimAPI_MakeTorus
from OCC.gp import *
from OCC.BRep import BRep_Tool, BRep_Builder
from OCC.BRepFilletAPI import BRepFilletAPI_MakeFillet
from OCC.GC import GC_MakeArcOfCircle, GC_MakeSegment
from OCC.BRepOffsetAPI import BRepOffsetAPI_MakeThickSolid, BRepOffsetAPI_ThruSections

from OCCUtils import Topo
from OCC.Bnd import Bnd_Box
import OCC
from OCC.Geom import *
from OCC.Geom2d import *
from OCC.GCE2d import *
import OCC.BRepLib as BRepLib


import OCCUtils
import time
import math

#http://blog.simengr.com/blogs/tutorials/webgl/pythonocc

myWidth = 50
myHeight = 70
myThickness = 30

aPnt1 = OCC.gp.gp_Pnt(-myWidth / 2.0, 0, 0)
aPnt2 = OCC.gp.gp_Pnt(-myWidth / 2.0, -myThickness / 4.0, 0)
aPnt3 = OCC.gp.gp_Pnt(0, -myThickness / 2.0, 0)
aPnt4 = OCC.gp.gp_Pnt(myWidth / 2.0, -myThickness / 4.0, 0)
aPnt5 = OCC.gp.gp_Pnt(myWidth / 2.0, 0, 0)


aArcOfCircle = GC_MakeArcOfCircle(aPnt2,aPnt3,aPnt4)
aSegment1 = GC_MakeSegment(aPnt1, aPnt2)
aSegment2 = GC_MakeSegment(aPnt4, aPnt5)

aEdge1 = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(aSegment1.Value())
aEdge2 = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(aArcOfCircle.Value())
aEdge3 = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(aSegment2.Value())

aWire = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire(aEdge1.Edge(), aEdge2.Edge(), aEdge3.Edge())

aOrigin = gp_Pnt(0, 0, 0)
xDir = gp_Dir(1, 0, 0)
xAxis = OCC.gp.gp_Ax1(aOrigin, xDir)

aTrsf = gp_Trsf()
aTrsf.SetMirror(xAxis)
aBRepTrsf = BRepBuilderAPI_Transform(aWire.Wire(), aTrsf)
aMirroredShape = aBRepTrsf.Shape()
aMirroredWire = topods.Wire(aMirroredShape)

mkWire = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire()
mkWire.Add(aWire.Wire())
mkWire.Add(aMirroredWire)
myWireProfile = mkWire.Wire()

myFaceProfile = BRepBuilderAPI_MakeFace(myWireProfile)
aPrismVec = gp_Vec(0, 0, myHeight)

myBody = OCC.BRepPrimAPI.BRepPrimAPI_MakePrism(myFaceProfile.Face(), aPrismVec)

mkFillet = BRepFilletAPI_MakeFillet(myBody.Shape())

anEdgeExplorer = OCC.TopExp.TopExp_Explorer(myBody.Shape(), OCC.TopAbs.TopAbs_EDGE)

while anEdgeExplorer.More():
    anEdge = topods.Edge(anEdgeExplorer.Current())
    # Add edge to fillet algorithm
    # ...
    mkFillet.Add(myThickness / 12.0, anEdge)
    anEdgeExplorer.Next()

myBody = mkFillet.Shape()

neckLocation = gp_Pnt(0, 0, myHeight)
neckAxis = OCC.gp.gp_DZ()
neckAx2 = gp_Ax2(neckLocation, neckAxis)

myNeckRadius = myThickness / 4.0
myNeckHeight = myHeight / 10.0
MKCylinder = OCC.BRepPrimAPI.BRepPrimAPI_MakeCylinder(neckAx2, myNeckRadius, myNeckHeight)
myNeck = MKCylinder.Shape()

myBody = OCC.BRepAlgoAPI.BRepAlgoAPI_Fuse(myBody, myNeck)


aFaceExplorer = OCC.TopExp.TopExp_Explorer(myBody.Shape(), OCC.TopAbs.TopAbs_FACE)
z_max = -1
top_face = None
while aFaceExplorer.More():
    bbox = Bnd_Box()
    aFace = topods.Face(aFaceExplorer.Current())
    OCC.BRepBndLib.brepbndlib_Add(aFace, bbox)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    y_mid = (xmin+xmax)/2
    if zmin+zmax>z_max:
        z_max = zmin+zmax
        top_face = aFace
#    aSurface = OCC.BRep.Tool.BRep_Tool_Surface(aFace)
    aFaceExplorer.Next()

facesToRemove = OCC.TopTools.TopTools_ListOfShape()
facesToRemove.Append(top_face)

myBody = BRepOffsetAPI_MakeThickSolid(myBody.Shape(), facesToRemove, -myThickness / 50, 1.0E-3)

neckAx2_bis = gp_Ax3(neckLocation , neckAxis)

aCyl1 = Geom_CylindricalSurface(neckAx2_bis, myNeckRadius * 0.99)
aCyl2 = Geom_CylindricalSurface(neckAx2_bis, myNeckRadius * 1.05)

aPnt = gp_Pnt2d(2.0 * math.pi, myNeckHeight / 2.0)
aDir = gp_Dir2d(2.0 * math.pi, myNeckHeight / 4.0)
anAx2d = gp_Ax2d(aPnt, aDir)

aMajor = 2.0 * math.pi
aMinor = myNeckHeight / 10.0
anEllipse1 = Geom2d_Ellipse(anAx2d, aMajor, aMinor)
anEllipse2 = Geom2d_Ellipse(anAx2d, aMajor, aMinor / 4.0)

anArc1 = Geom2d_TrimmedCurve(anEllipse1.GetHandle(), 0, math.pi)
anArc2 = Geom2d_TrimmedCurve(anEllipse2.GetHandle(), 0, math.pi)

anEllipsePnt1 = anEllipse1.Value(0)
anEllipsePnt2 = anEllipse1.Value(math.pi)
aSegment = GCE2d_MakeSegment(anEllipsePnt1, anEllipsePnt2)

anEdge1OnSurf1 = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(anArc1.GetHandle(), aCyl1.GetHandle())
anEdge2OnSurf1 = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(aSegment.Value(), aCyl1.GetHandle())
anEdge1OnSurf2 = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(anArc2.GetHandle(), aCyl2.GetHandle())
anEdge2OnSurf2 = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(aSegment.Value(), aCyl2.GetHandle())

threadingWire1 = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire(anEdge1OnSurf1.Edge(), anEdge2OnSurf1.Edge())
threadingWire2 = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire(anEdge1OnSurf2.Edge(), anEdge2OnSurf2.Edge())

BRepLib.breplib.BuildCurves3d(threadingWire1.Shape())
BRepLib.breplib.BuildCurves3d(threadingWire2.Shape())

aTool = BRepOffsetAPI_ThruSections(True)
aTool.AddWire(threadingWire1.Wire())
aTool.AddWire(threadingWire2.Wire())
aTool.CheckCompatibility(False)
myThreading = aTool.Shape()

aRes = TopoDS_Compound()
aBuilder = BRep_Builder()
aBuilder.MakeCompound(aRes)
aBuilder.Add(aRes, myBody.Shape())
aBuilder.Add(aRes, myThreading)

display, start_display, add_menu, add_function_to_menu = init_display()
display.EraseAll()
display.DisplayShape(aRes, update=True)

display.FitAll()

start_display()

