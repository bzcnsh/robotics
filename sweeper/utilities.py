from OCCUtils.Topology import WireExplorer
from OCC.Bnd import Bnd_Box

import OCC.BRepBuilderAPI
import OCC.BRep
import OCC.TopoDS
import OCCUtils
import OCC.TopTools
import OCC.gp
import OCC.BRep
import OCC.BRepExtrema

import pprint
import time
import math
import scipy.linalg
import numpy

class surface_sweeper:
    def __init__(self, parameters={}):
        if parameters.has_key('sweep_width'): self.sweep_width = parameters['sweep_width']
        if parameters.has_key('max_vertex_variance'): self.max_vertex_variance = parameters['max_vertex_variance']
        if parameters.has_key('max_sweep_line_variance'): self.max_sweep_line_variance = parameters['max_sweep_line_variance']
        if parameters.has_key('sweep_orientation'): self.sweep_orientation = parameters['sweep_orientation']
        if parameters.has_key('sweep_surface'): self.sweep_surface = parameters['sweep_surface']
        if parameters.has_key('wire_join_max_distance'): self.wire_join_max_distance = parameters['wire_join_max_distance']
    
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
                time.sleep(0.1)
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
            if self.sweep_orientation=="x":
                vertex_hash[pnt.X()]=v
            if self.sweep_orientation=="y":
                vertex_hash[pnt.Y()]=v
            if self.sweep_orientation=="z":
                vertex_hash[pnt.Z()]=v
        for k in sorted(vertex_hash.keys()):
            vertices.append(vertex_hash[k])
        return vertices

    def make_edge_from_vertices(self, vertice1, vertice2):
        make_edge = OCC.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(vertice1, vertice2)
        while not make_edge.IsDone():
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

    def is_equal_point(self, p1, p2):
        distance = math.sqrt( math.pow(p1[0]-p2[0], 2) + math.pow(p1[1]-p2[1], 2) + math.pow(p1[2]-p2[2], 2))
        return distance<self.max_vertex_variance
    
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
            if self.sweep_orientation=="x":
                wires_hash[x]=i
            if self.sweep_orientation=="y":
                wires_hash[y]=i
            if self.sweep_orientation=="z":
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
        if level==0:
            #find the first common vertices between whole_shape and part_shape
            #have to use vertices taken from whole_shape for Topo functions such as faces_from_vertex(v)
            for v_p in topo_p.vertices():
                for v_w in topo_w.vertices():
                    if self.is_equal_vertex(v_p, v_w):
                        #use v_w as our proxy in topo_w
                        part_vertices = [v_w]
                        break
        if level>0:
            part_vertices = topo_p.vertices()
        if part_vertices:
            for v in part_vertices:
                if not v.__hash__() in vertices:
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
        wires = []
        #if an edge is connected to a wire, add the edge to the wire
        #else, create a new wire with the edge
        for e in edges:
            found_edge_in_wire = False
            for i in range(0, len(wires)):
                edges_in_wire = self.get_connected_shapes(wires[i], e, "edge")
                if len(edges_in_wire)>0:
                    edges_in_wire.append(e)
                    wires[i]=self.make_wire_from_edges(edges_in_wire)
                    found_edge_in_wire = True
                    break
            if not found_edge_in_wire:
                wires.append(self.make_wire_from_edges([e]))
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
        if direction=="begin":
            vertices = self.get_ordered_vertices_from_wire(wire)
            begin_point0 = self.get_point_from_vertex(vertices[0])
            begin_point1 = self.get_point_from_vertex(vertices[1])
            extension_vertex = self.make_vertex_from_point(self.get_extension_point(begin_point1, begin_point0, length))
            vertices[0] = extension_vertex
            return self.make_wire_from_vertices(vertices)
        if direction=="end":
            vertices = self.get_ordered_vertices_from_wire(wire)
            end_point0 = self.get_point_from_vertex(vertices[len(vertices)-1]) 
            end_point1 = self.get_point_from_vertex(vertices[len(vertices)-2]) 
            extension_vertex = self.make_vertex_from_point(self.get_extension_point(end_point1, end_point0, length))
            vertices[len(vertices)-1] = extension_vertex 
            return self.make_wire_from_vertices(vertices)
        if direction=="both":
            wire = self.extend_wire(wire, length, "begin")
            wire = self.extend_wire(wire, length, "end")
            return wire
