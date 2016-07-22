import OCC.BRepBuilderAPI
import OCC.BRepOffsetAPI
import OCC.BRep
import OCC.TopoDS
import OCCUtils
import OCC.TopTools
import OCC.gp
import OCC.BRep
import OCC.BRepExtrema
import OCC.Bnd
from OCC.BRepGProp import BRepGProp_Face

import pprint
import time
import math
import scipy.linalg
import numpy
import pickle

class surface_sweeper:
    def __init__(self, parameters={}):
        if parameters.has_key('sweep_width'): self.sweep_width = parameters['sweep_width']
        if parameters.has_key('max_vertex_variance'): self.max_vertex_variance = parameters['max_vertex_variance']
        if parameters.has_key('max_sweep_line_variance'): self.max_sweep_line_variance = parameters['max_sweep_line_variance']
        if parameters.has_key('sweep_direction'): self.sweep_direction = parameters['sweep_direction']
        if parameters.has_key('slice_direction'): self.slice_direction = parameters['slice_direction']
        if parameters.has_key('sweep_surface'): self.sweep_surface = parameters['sweep_surface']
        if parameters.has_key('wire_join_max_distance'): self.wire_join_max_distance = parameters['wire_join_max_distance']
        if parameters.has_key('path_extension_distance'): self.path_extension_distance = parameters['path_extension_distance']
        if parameters.has_key('object_center'): self.object_center = parameters['object_center']

    #reduce number of vertices on a wire, to smoothen robotic arm's movement
    def reduce_wire_edge(self, wire):
        #a vertex can be removed if it's sufficiently close to the line between the two vertices on its side
        #start from one end, move to the other end
        wire_keep = []
        old_wire_topo = OCCUtils.Topo(wire)
        vertices = self.get_ordered_vertices_from_wire(wire)
        long_edge_start = 0
        wire_keep.append(long_edge_start)
        #long edge is between vertices[long_edge_start] and vertices[i]
        #vertices between the two end vertices are tested for distance to long edge
        #if a vertice is too far from the long edge, then the long edge's end vertex is moved backward
        long_edge_end = long_edge_start+2
        skippable = []
        while long_edge_end < len(vertices):
            new_edge = self.make_edge_from_vertices(vertices[long_edge_start], vertices[long_edge_end])
            too_far = False
            for j in range(long_edge_start+1, long_edge_end):
                distance = self.get_nearest(vertices[j], new_edge)
                if distance.Value()>self.max_sweep_line_variance:
                    if skippable:
                        long_edge_start = max(skippable)+1 if max(skippable)+1> j else j
                    else:
                        long_edge_start = j
                    long_edge_end=long_edge_start+2
                    wire_keep.append(long_edge_start)
                    too_far=True
                    break
            if not too_far:
                skippable.append(long_edge_end-1)
                #all vertices inside the long edge can be removed
                long_edge_end = long_edge_end + 1
        #add the last vertex
        wire_keep.append(len(vertices)-1)
        #build new wire from these vertises
        new_wire = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire()
        occ_seq = OCC.TopTools.TopTools_ListOfShape()
        for i in range(len(wire_keep)-1):
            t11 = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(vertices[wire_keep[i]], vertices[wire_keep[i+1]])
            while not t11.IsDone():
                time.sleep(0.01)
            new_edge = t11.Edge()
            occ_seq.Append(new_edge)
        new_wire.Add(occ_seq)
        return new_wire.Wire()

    def get_nearest(self, shape1, shape2):
        distance = OCC.BRepExtrema.BRepExtrema_DistShapeShape()
        distance.LoadS1(shape1)
        distance.LoadS2(shape2)
        distance.Perform()
        while not distance.IsDone():
            time.sleep(0.01)
        return distance

    def get_ordered_vertices_from_wire(self, wire):
        vertices = []
        vertex_hash = {}
        wire_topo=OCCUtils.Topo(wire)
        brt = OCC.BRep.BRep_Tool()
        for v in wire_topo.vertices():
            pnt = brt.Pnt(OCC.TopoDS.topods_Vertex(v))
            if self.sweep_direction=="x":
                vertex_hash[pnt.X()]=v
            if self.sweep_direction=="y":
                vertex_hash[pnt.Y()]=v
            if self.sweep_direction=="z":
                vertex_hash[pnt.Z()]=v
        for k in sorted(vertex_hash.keys()):
            vertices.append(vertex_hash[k])
        return vertices

    def make_edge_from_vertices(self, vertice1, vertice2):
        make_edge = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(vertice1, vertice2)
        while not make_edge.IsDone():
            print("make_edge_from_vertices")
            OCCUtils.Topology.dumpTopology(vertice1)
            OCCUtils.Topology.dumpTopology(vertice2)
            time.sleep(0.1)
        return make_edge.Edge()

    def make_vertex_from_point(self, point):
        return OCC.BRepBuilderAPI.BRepBuilderAPI_MakeVertex(OCC.gp.gp_Pnt(point[0], point[1], point[2])).Vertex()
    
    def make_edge_from_points(self, p1, p2=None):
        if p2:
            vertice1 = self.make_vertex_from_point(p1)
            vertice2 = self.make_vertex_from_point(p2)
        else:
            vertice1 = self.make_vertex_from_point(p1[0])
            vertice2 = self.make_vertex_from_point(p1[1])
        return self.make_edge_from_vertices(vertice1, vertice2)
        
    def make_wire_from_point_list(self, points):
        make_wire = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire()
        occ_seq = OCC.TopTools.TopTools_ListOfShape()
        for i in range(0, len(points)-1):
            occ_seq.Append(self.make_edge_from_points(points[i], points[i+1]))
        make_wire.Add(occ_seq)
        return make_wire.Wire()

    def make_wire_from_vertices(self, vertices):
        make_wire = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire()
        occ_seq = OCC.TopTools.TopTools_ListOfShape()
        for i in range(0, len(vertices)-1):
            occ_seq.Append(self.make_edge_from_vertices(vertices[i], vertices[i+1]))
        make_wire.Add(occ_seq)
        return make_wire.Wire()

    def make_wire_from_edges(self, edges):
        make_wire = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire()
        occ_seq = OCC.TopTools.TopTools_ListOfShape()
        for i in edges:
            occ_seq.Append(i)
        make_wire.Add(occ_seq)
        return make_wire.Wire()

    def get_point_from_vertex(self, vertex):
        brt = OCC.BRep.BRep_Tool()
        pnt = brt.Pnt(OCC.TopoDS.topods_Vertex(vertex))
        return (pnt.X(), pnt.Y(), pnt.Z())

    def get_distance_vertices(self, v1, v2):
        return self.get_distance_points(self.get_point_from_vertex(v1), self.get_point_from_vertex(v2)) 
        
    def get_distance_points(self, p1, p2):
        return math.sqrt( math.pow(p1[0]-p2[0], 2) + math.pow(p1[1]-p2[1], 2) + math.pow(p1[2]-p2[2], 2))

    def is_equal_point(self, p1, p2):
        return self.get_distance_points(p1, p2)<self.max_vertex_variance
    
    def is_equal_vertex(self, v1, v2):
        return self.is_equal_point(self.get_point_from_vertex(v1), self.get_point_from_vertex(v2))
    
    def is_equal_edge(self, e1, e2):
        vertices_e1 = list(OCCUtils.Topo(e1).vertices())
        assert len(vertices_e1)==2, "e1 is not a valid edge"
        vertices_e2 = list(OCCUtils.Topo(e2).vertices())
        assert len(vertices_e2)==2, "e2 is not a valid edge"
        return self.contain_vertex(e1, vertices_e2[0]) and self.contain_vertex(e1, vertices_e2[1])

    def is_equal_wire(self, w1, w2):
        edges_w1 = list(OCCUtils.Topo(w1).edges())
        edges_w2 = list(OCCUtils.Topo(w2).edges())
        if len(edges_w1) != len(edges_w2):
            return False
        for e in edges_w1:
            if not self.contain_edge(w2, e):
                return False
        return True

    def contain_vertex(self, shape, vertex):
        topo = OCCUtils.Topo(shape)
        for v in topo.vertices():
            if self.is_equal_vertex(v, vertex):
                return True
        return False
    
    def count_vertices(self, shape):
        topo = OCCUtils.Topo(shape)
        return len(list(topo.vertices()))
        
    def count_edges(self, shape):
        topo = OCCUtils.Topo(shape)
        return len(list(topo.edges()))
        
    def contain_edge(self, shape, edge):
        vertices_e = list(OCCUtils.Topo(edge).vertices())
        topo_s = OCCUtils.Topo(shape)
        for e in topo_s.edges():
            if self.contain_vertex(e, vertices_e[0]) and self.contain_vertex(e, vertices_e[1]):
                return True
        return False

    def join_nearby_edges(self, wires):
        #expect wires to be non-overlapping in the direction sweep-orientation
        if len(wires)<=1:
            return wires
        ordered_vertices = []
        ordered_wires = []
        wires_hash = {}
        #sort vertices with each wire 
        for w in wires:
            ordered_vertices.append(self.get_ordered_vertices_from_wire(w))
        #sort wires by its first vertex
        for i in ordered_vertices:
            x, y, z = self.get_point_from_vertex(i[0])
            if self.sweep_direction=="x":
                wires_hash[x]=i
            if self.sweep_direction=="y":
                wires_hash[y]=i
            if self.sweep_direction=="z":
                wires_hash[z]=i
         
        for k in sorted(wires_hash.keys()):
            ordered_wires.append(wires_hash[k])
        merged = True
        while merged:
            merged = False
            for i in range(0, len(ordered_wires)-1):
                distance = self.get_nearest(ordered_wires[i][len(ordered_wires[i])-1], ordered_wires[i+1][0])
                if distance.Value()<self.wire_join_max_distance:
                    #remove the end vertices and join the two wires together
                    del ordered_wires[i][-1]
                    del ordered_wires[i+1][0]
                    ordered_wires[i] = ordered_wires[i] + ordered_wires[i+1]
                    del ordered_wires[i+1]
                    merged = True
                    break
        #sometimes a section compound can have multiple edges with the same vertex
        for i in range(0, len(ordered_wires)):
            delete_dup = True
            while(delete_dup):
                delete_dup = False
                for j in range(0, len(ordered_wires[i])-1):
                    if self.is_equal_vertex(ordered_wires[i][j], ordered_wires[i][j+1]):
                        del ordered_wires[i][j]
                        delete_dup = True
                        break
        output_wires = map(self.make_wire_from_vertices, ordered_wires)
        return output_wires
    
    def get_connected_shapes(self, whole_shape, part_shape, shape_type, hash_shapes=None, topo_w=None, vertices=None, level=0):
        #whole_shape can have multiple separate faces
        #return a list of all shapes connected to part_shape (through shared vertices, whether directly or indirectly
        if not topo_w: topo_w = OCCUtils.Topo(whole_shape)
        if not vertices: vertices = {}
        if not hash_shapes: hash_shapes={}
        topo_p = OCCUtils.Topo(part_shape)
        new_shapes = {}
        part_vertices = None
        #print("get_connected_shapes level %i" % level)
        #find the first common vertices between whole_shape and part_shape
        #have to use vertices taken from whole_shape for Topo functions such as faces_from_vertex(v)
        proxy_vertices = []
        for v_p in topo_p.vertices():
            for v_w in topo_w.vertices():
                if self.is_equal_vertex(v_p, v_w):
                    #use v_w as our proxy in topo_w
                    if not vertices.has_key(v_w.__hash__()):
                        proxy_vertices.append(v_w)
        for v in proxy_vertices:
            if not vertices.has_key(v.__hash__()):
                vertices[v.__hash__()]=1
                if shape_type=="face":
                    immediately_connected_shapes = topo_w.faces_from_vertex(v)
                if shape_type=="edge":
                    immediately_connected_shapes = topo_w.edges_from_vertex(v)
                for s in immediately_connected_shapes:
                    h = s.__hash__()
                    if not h in hash_shapes:
                        hash_shapes[h] = 1
                        new_shapes[h]=s
                    
        for h, s in new_shapes.items():
            self.get_connected_shapes(whole_shape, s, shape_type, hash_shapes, topo_w, vertices, level+1)
        shape_list = []
        if level==0:
            if hash_shapes:
                if shape_type=="face":
                    all_shapes = topo_w.faces()
                if shape_type=="edge":
                    all_shapes = topo_w.edges()
                for s in all_shapes:
                    if hash_shapes.has_key(s.__hash__()):
                        shape_list.append(s)
        return shape_list

    def get_wires_from_edges(self, edges):
        wires= []
        #turn every edge to a wire, then keep joining connected wires
        for e in edges:
            wires.append(self.make_wire_from_edges([e]))
        joined_wires = True
        while joined_wires:
            joined_wires = False
            for w1 in wires:
                for w2 in wires:
                    if w1!=w2:
                        edges_in_wire = self.get_connected_shapes(w1, w2, "edge")
                        if len(edges_in_wire)>0:
                            topo_1 = OCCUtils.Topo(w1)
                            topo_2 = OCCUtils.Topo(w2)
                            edges1=list(topo_1.edges())
                            edges2=list(topo_2.edges())
                            new_wire = self.make_wire_from_edges(edges1+edges2)
                            topo_new = OCCUtils.Topo(new_wire)
                            edges_new = list(topo_new.edges())
                            if len(edges1)+len(edges2)!=len(edges_new):
                                #do not join if edge count change
                                continue
                            wires.append(new_wire)
                            wires.remove(w1)
                            wires.remove(w2)
                            joined_wires = True
                            break
                if joined_wires:
                    break
        return wires
    
    def get_extension_point(self, point0, point1, length):
        mat_p0 = numpy.mat(point0)
        mat_p1 = numpy.mat(point1)
        direction=mat_p1-mat_p0
        direction=direction/scipy.linalg.norm(direction)
        mat_extension = mat_p1 + direction * length
        return mat_extension.A[0]
        
    def extend_wire(self, wire, length, direction="both"):
        #extend the first and last edge of a wire
        if length<=0: return wire 
        if direction=="begin":
            vertices = self.get_ordered_vertices_from_wire(wire)
            begin_point0 = self.get_point_from_vertex(vertices[0])
            begin_point1 = self.get_point_from_vertex(vertices[1])
            extension_vertex = self.make_vertex_from_point(self.get_extension_point(begin_point1, begin_point0, length))
            vertices.insert(0, extension_vertex)
            return self.make_wire_from_vertices(vertices)
        if direction=="end":
            vertices = self.get_ordered_vertices_from_wire(wire)
            end_point0 = self.get_point_from_vertex(vertices[len(vertices)-1]) 
            end_point1 = self.get_point_from_vertex(vertices[len(vertices)-2]) 
            extension_vertex = self.make_vertex_from_point(self.get_extension_point(end_point1, end_point0, length))
            vertices.append(extension_vertex) 
            return self.make_wire_from_vertices(vertices)
        if direction=="both":
            wire = self.extend_wire(wire, length, "begin")
            wire = self.extend_wire(wire, length, "end")
            return wire

    def get_strip_boundary(self, aShape, spine):
        pipe = OCC.BRepOffsetAPI.BRepOffsetAPI_MakePipeShell(spine)
        #pipe.SetTransitionMode(OCC.BRepBuilderAPI.BRepBuilderAPI_RoundCorner)
        for v in self.get_ordered_vertices_from_wire(spine):
            brt = OCC.BRep.BRep_Tool()
            pnt = brt.Pnt(v)
            circle = OCC.gp.gp_Circ(OCC.gp.gp_Ax2(pnt, OCC.gp.gp_DZ()), self.sweep_width)
            profile_edge = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeWire(OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circle).Edge())
            pipe.Add(profile_edge.Shape(), False, True)
            break
        pipe.Build()
        while not pipe.IsDone():
            print("waiting for pipe")
            time.sleep(0.01)
        section = OCC.BRepAlgoAPI.BRepAlgoAPI_Section(pipe.Shape(), aShape)
        #section.Approximation(False)
        section.Build()
        while not section.IsDone():
            print("getStripBoundary", "waiting for section")
            time.sleep(0.01)
        return section

    def sweep_face(self, aFace, initial_section, up_or_down):
        #start with the initial section
        sweep_wires = []
        sections = [initial_section]
        min_last=-99999999
        max_last=99999999
        #one sweep can have multiple segments, each segment has its own spine wire
        #a spine wire produces one or two intersection with the surface (above and below the spine)
        #the above and below sections are mixed in a "compound" shape, each section can have multiple wires, due to underlying software bug, wires belong to same section should be merged
        #distance between the above and below sections is 2 x sweep_width
        while sections:
            section_wires = []
# take dump for specific problem
#            for s in sections:
#                p = [-448.702808548, -866.726174489, -208.484265004]
#                v = self.make_vertex_from_point(p)
#                if self.contain_vertex(s.Shape(), v):
#                    pickle.dump(s.Shape(), open( "sweep_face_001_section.dmp", "wb" ) )
#                    assert False, "took dump"
#                    break

            proper_side_section_wires = []
            joined_section_wires = []
            for s in sections:
                section_wires = section_wires + self.get_wires_from_edges(list(OCCUtils.Topo(s.Shape()).edges()))
            #a pipe intersects with surface above and below the pipe's spine line, the minimum distance between the two section lines sweep_width, or 2xpipe raidus
            #compare wire with previous section wire
            for w in section_wires:
                xmin, ymin, zmin, xmax, ymax, zmax = self.get_shape_boundary([w])
                #print("zmin: %f, zmax: %f" % (zmin, zmax))
                if ( (self.slice_direction=='x' and ((up_or_down=='up' and xmin>min_last) or (up_or_down=='down' and xmax<max_last))) or
                     (self.slice_direction=='y' and ((up_or_down=='up' and ymin>min_last) or (up_or_down=='down' and ymax<max_last))) or
                     (self.slice_direction=='z' and ((up_or_down=='up' and zmin>min_last) or (up_or_down=='down' and zmax<max_last))) ):
                    proper_side_section_wires.append(w)
            if not proper_side_section_wires:
                break
            #print("proper side section_wire count 02: %i" % len(proper_side_section_wires))
            joined_section_wires = self.join_nearby_edges(proper_side_section_wires)
            #print("joined section_wire count 03: %i" % len(joined_section_wires))
            xmin, ymin, zmin, xmax, ymax, zmax = self.get_shape_boundary(joined_section_wires)
            if self.slice_direction=='x':
                min_last = xmin
                max_last = xmax
            if self.slice_direction=='y':
                min_last = ymin
                max_last = ymax
            if self.slice_direction=='z':
                min_last = zmin
                max_last = zmax
            print("direction: %s, xmin: %f, ymin: %f, zmin: %f, xmax: %f, ymax: %f, zmax: %f" % (up_or_down, xmin, ymin, zmin, xmax, ymax, zmax))
        
            sections = []
            for wire in joined_section_wires:
                wire = self.reduce_wire_edge(wire)
                extended_wires = self.extend_wire(wire, self.path_extension_distance, "both")
                section = self.get_strip_boundary(aFace, extended_wires)
                if section:
                    sections.append(section)
            sweep_wires = sweep_wires + joined_section_wires
        return sweep_wires

    def get_shape_boundary(self, shapes):
        bbox = OCC.Bnd.Bnd_Box()
        for s in shapes:
            OCC.BRepBndLib.brepbndlib_Add(s, bbox)
        return bbox.Get()

    def get_slices(self, shape, delta):
        xmin, ymin, zmin, xmax, ymax, zmax = self.get_shape_boundary([shape])
        direction = self.slice_direction
        if direction=="x":
            D = OCC.gp.gp_Dir(1., 0., 0.)  # the x direction
        if direction=="y":
            D = OCC.gp.gp_Dir(0., 1., 0.)  # the y direction
        if direction=="z":
            D = OCC.gp.gp_Dir(0., 0., 1.)  # the z direction
        # Perform slice
        sections = []
        if direction=="x":
            iMax = int((xmax-xmin)/delta)
        if direction=="y":
            iMax = int((ymax-ymin)/delta)
        if direction=="z":
            iMax = int((zmax-zmin)/delta)
        for i in range(iMax):
            # Create Plane defined by a point and the perpendicular direction
            if direction=="x":
                #0.5 so the first slice is not at edge
                x = xmin+(i+0.5)*delta
                Pln = OCC.gp.gp_Pln(OCC.gp.gp_Pnt(x, 0, 0), D)
            if direction=="y":
                y = ymin+(i+0.5)*delta
                Pln = OCC.gp.gp_Pln(OCC.gp.gp_Pnt(0, y, 0), D)
            if direction=="z":
                z = zmin+(i+0.5)*delta
                Pln = OCC.gp.gp_Pln(OCC.gp.gp_Pnt(0, 0, z), D)
            face = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeFace(Pln).Shape()
            # Computes Shape/Plane intersection
            section = OCC.BRepAlgoAPI.BRepAlgoAPI_Section(shape, face)
            if section.IsDone():
                sections.append(section)
        return sections

    def get_face_normal(self, face):
        bf = OCC.BRepGProp.BRepGProp_Face(face)
        bounds = bf.Bounds()
        vec = OCC.gp.gp_Vec()
        pt = OCC.gp.gp_Pnt()
        #get a normal vector to the face
        bf.Normal(bounds[0],bounds[1],pt,vec)
        direction = OCC.gp.gp_Dir(vec)
        return direction