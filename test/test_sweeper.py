import sys

sys.path.append('../')
import unittest
import sweeper.utilities as ut
import pprint
import pickle

class UtilitiesTestCase(unittest.TestCase):
    def setUp(self):
        self.sweeper=ut.surface_sweeper({'sweep_width': 60, 'max_vertex_variance': 0.001, 'sweep_orientation': 'y', 'wire_join_max_distance': 10.0, 'max_sweep_line_variance': 0.5})
    def tearDown(self):
        print("tear down")
    def test_make_vertex_from_point(self):
        p = [1.0, 2.5, 3.25]
        vertex = self.sweeper.make_vertex_from_point(p)
        rp = self.sweeper.get_point_from_vertex(vertex)
        assert self.sweeper.is_equal_point(rp, p), "is_equal_point error" 
    def test_is_equal_vertex(self):
        p = [1.0, 2.5, 3.25]
        v1 = self.sweeper.make_vertex_from_point(p)
        v2 = self.sweeper.make_vertex_from_point(p)
        assert self.sweeper.is_equal_vertex(v1, v2), "is_equal_vertex error" 
    def test_make_edge_from_points(self):
        p = [[1.0, 2.5, 3.25], [0.0, 0.0, 2.0]]
        edge = self.sweeper.make_edge_from_points(p[0], p[1])
        v1 = self.sweeper.make_vertex_from_point(p[0])
        v2 = self.sweeper.make_vertex_from_point(p[1])
        return self.sweeper.contain_vertex(edge, v1) and self.sweeper.contain_vertex(edge, v2)
     
    def test_is_equal_edge(self):
        p = [[1.0, 2.5, 3.25], [0.0, 0.0, 2.0]]
        t = [[2.0, 2.5, 3.25], [0.0, 0.0, 3.0]]
        edge1 = self.sweeper.make_edge_from_points(p[0], p[1])
        edge2 = self.sweeper.make_edge_from_points(p[1], p[0])
        assert self.sweeper.is_equal_edge(edge1, edge2), "swapping vertices should result in same edge"
        edge3 = self.sweeper.make_edge_from_points(t[0], t[1])
        assert not self.sweeper.is_equal_edge(edge1, edge3), "different vertices creates different edges"
        
    def test_make_wire_from_point_list(self):
        p1 = [[1.0, 2.5, 3.25], [0.0, 0.0, 2.0], [2.0, 5.0, 2.0]]
        p2 = [[2.0, 5.0, 2.0],  [0.0, 0.0, 2.0], [1.0, 2.5, 3.25]]
        p3 = [[2.0, 5.0, 2.0],  [0.0, 0.0, 2.0], [4.0, 2.5, 3.25]]
        wire1 = self.sweeper.make_wire_from_point_list(p1)
        wire2 = self.sweeper.make_wire_from_point_list(p2)
        wire3 = self.sweeper.make_wire_from_point_list(p3)
        assert self.sweeper.is_equal_wire(wire1, wire2), "swapping edges should not change a wire"
        assert not self.sweeper.is_equal_wire(wire1, wire3), "different vertices result in different wire"
        
    def test_make_wire_from_vertices(self):
        p1 = [[1.0, 2.5, 3.25], [0.0, 0.0, 2.0], [2.0, 5.0, 2.0]]
        p2 = [[2.0, 5.0, 2.0],  [0.0, 0.0, 2.0], [4.0, 2.5, 3.25]]
        v1 = map(self.sweeper.make_vertex_from_point, p1)
        v2 = map(self.sweeper.make_vertex_from_point, p2)
        wire1 = self.sweeper.make_wire_from_vertices(v1)
        wire1_2 = self.sweeper.make_wire_from_vertices([v1[2], v1[1], v1[0]])
        wire2 = self.sweeper.make_wire_from_vertices(v2)
        assert self.sweeper.is_equal_wire(wire1, wire1_2), "swapping edges should not change a wire"
        assert not self.sweeper.is_equal_wire(wire1, wire2), "different vertices result in different wire"

    def test_make_wire_from_edges(self):
        p1 = [[1.0, 2.5, 3.25], [0.0, 0.0, 2.0], [2.0, 5.0, 2.0]]
        p2 = [[2.0, 5.0, 2.0],  [0.0, 0.0, 2.0], [4.0, 2.5, 3.25]]
        e1 = self.sweeper.make_edge_from_points(p1[0], p1[1])
        e2 = self.sweeper.make_edge_from_points(p1[1], p1[2])
        e3 = self.sweeper.make_edge_from_points(p2[0], p2[1])
        e4 = self.sweeper.make_edge_from_points(p2[1], p2[2])
        wire1 = self.sweeper.make_wire_from_edges([e1, e2])
        wire2 = self.sweeper.make_wire_from_edges([e2, e1])
        wire3 = self.sweeper.make_wire_from_edges([e3, e4])
        assert self.sweeper.is_equal_wire(wire1, wire1), "swapping edges should not change a wire"
        assert not self.sweeper.is_equal_wire(wire1, wire3), "different vertices result in different wire"

    def test_get_ordered_vertices_from_wire(self):
        setattr(self.sweeper, 'sweep_orientation', 'x')
        p = [[16.0, 2.5, 3.25], [14.0, 0.0, 2.0], [12.0, 5.0, 2.0], [10.0, 5.0, 2.0], [8.0, 5.0, 2.0], [7.0, 5.0, 2.0]]
        v = map(self.sweeper.make_vertex_from_point, p)
        wire = self.sweeper.make_wire_from_point_list(p)
        vertices = self.sweeper.get_ordered_vertices_from_wire(wire)
        for i in range(1, len(p)):
            assert self.sweeper.is_equal_vertex(vertices[i], v[len(v)-i-1]), "sorting by x will reverse p's order"

    def test_reduce_wire_edge(self):
        setattr(self.sweeper, 'sweep_orientation', 'x')
        p = [[0.0,  0.0,    0.0],
             [1.0,  1.0,    0.0],
             [2.0,  -2.0,    0.0],
             [3.0,  0.0,    0.0],
             [4.0,  1.0,    0.0],
             [5.0,  -1.0,    0.0],
             [6.0,  0.0,    0.0],
             [7.0,  -1.0,    0.0],
             [8.0,  0.0,    0.0]]
        p_090 = [[0.0,  0.0,    0.0],
             [1.0,  1.0,    0.0],
             [2.0,  -2.0,    0.0],
             [4.0,  1.0,    0.0],
             [5.0,  -1.0,    0.0],
             [6.0,  0.0,    0.0],
             [7.0,  -1.0,    0.0],
             [8.0,  0.0,    0.0]]
        p_1_1 = [[0.0,  0.0,    0.0],
             [1.0,  1.0,    0.0],
             [2.0,  -2.0,    0.0],
             [4.0,  1.0,    0.0],
             [5.0,  -1.0,    0.0],
             [8.0,  0.0,    0.0]]
        p_027 = p
        p_028 = p_090
        w = self.sweeper.make_wire_from_point_list(p)
        w_090 = self.sweeper.make_wire_from_point_list(p_090)
        w_1_1 = self.sweeper.make_wire_from_point_list(p_1_1)
        w_027 = w
        w_028 = w_090
        setattr(self.sweeper, 'max_sweep_line_variance', 0.9)
        #import pdb; pdb.set_trace()
        r = self.sweeper.reduce_wire_edge(w)
        assert self.sweeper.is_equal_wire(w_090, r), "problem in reduce_wire_edge with variance==0.9"
        setattr(self.sweeper, 'max_sweep_line_variance', 0.27)
        r = self.sweeper.reduce_wire_edge(w)
        assert self.sweeper.is_equal_wire(w_027, r), "problem in reduce_wire_edge with variance==0.27"
        setattr(self.sweeper, 'max_sweep_line_variance', 0.28)
        r = self.sweeper.reduce_wire_edge(w)
        assert self.sweeper.is_equal_wire(w_028, r), "problem in reduce_wire_edge with variance==0.28"
    
    def test_join_nearby_edges(self):
        p=[
            [[0.0,  0.0,    0.0],
             [1.0,  1.0,    0.0],
             [2.0,  -2.0,    0.0],
             [3.0,  0.0,    0.0],
             [4.0,  1.0,    0.0],
             [5.0,  -1.0,    0.0],
             [6.0,  0.0,    0.0],
             [7.0,  -1.0,    0.0],
             [8.0,  0.0,    0.0]],
           
             [[8.0,  0.1,    0.1],
             [9.0,  1.0,    0.0],
             [10.0,  -2.0,    0.0],
             [11.0,  1.0,    0.0]],
           
             [[11.0,  1.1,    0.1],
             [12.0,  1.0,    0.0],
             [13.0,  -2.0,    0.0],
             [14.0,  1.0,    0.0],
             [15.0,  -1.0,    0.0],
             [16.0,  0.0,    0.0]],
           
             [[18.0,  1.1,    0.1],
             [19.0,  1.0,    0.0],
             [20.0,  -2.0,    0.0],
             [21.0,  1.0,    0.0],
             [22.0,  -1.0,    0.0],
             [23.0,  0.0,    0.0]],
           
             [[23.0,  0.1,    0.1],
             [24.0,  1.0,    0.0],
             [25.0,  -2.0,    0.0],
             [26.0,  1.0,    0.0],
             [27.0,  -1.0,    0.0],
             [28.0,  0.0,    0.0]]]

        r = [[[0.0,  0.0,    0.0],
             [1.0,  1.0,    0.0],
             [2.0,  -2.0,    0.0],
             [3.0,  0.0,    0.0],
             [4.0,  1.0,    0.0],
             [5.0,  -1.0,    0.0],
             [6.0,  0.0,    0.0],
             [7.0,  -1.0,    0.0],
             [9.0,  1.0,    0.0],
             [10.0,  -2.0,    0.0],
             [12.0,  1.0,    0.0],
             [13.0,  -2.0,    0.0],
             [14.0,  1.0,    0.0],
             [15.0,  -1.0,    0.0],
             [16.0,  0.0,    0.0]],
             
             [[18.0,  1.1,    0.1],
             [19.0,  1.0,    0.0],
             [20.0,  -2.0,    0.0],
             [21.0,  1.0,    0.0],
             [22.0,  -1.0,    0.0],
             [24.0,  1.0,    0.0],
             [25.0,  -2.0,    0.0],
             [26.0,  1.0,    0.0],
             [27.0,  -1.0,    0.0],
             [28.0,  0.0,    0.0]]]
        setattr(self.sweeper, 'wire_join_max_distance', 0.5)
        setattr(self.sweeper, 'sweep_orientation', 'x')
        wires = map(self.sweeper.make_wire_from_point_list, p)
        make_result = map(self.sweeper.make_wire_from_point_list, r)
        run_result = self.sweeper.join_nearby_edges(wires)
        assert len(make_result)==len(run_result), "make_result and run_result has different length"
        for i in range(0, len(make_result)):
            assert self.sweeper.is_equal_wire(make_result[i], run_result[i]), "join_nearby_edges unexpected result"
    
    def test_get_connected_shapes(self):
        p=[
            [[0.0,  0.0,    0.0],
             [1.0,  1.0,    0.0],
             [2.0,  -2.0,    0.0],
             [3.0,  0.0,    0.0],
             [4.0,  1.0,    0.0],
             [5.0,  -1.0,    0.0],
             [6.0,  0.0,    0.0],
             [7.0,  -1.0,    0.0],
             [8.0,  0.0,    0.0]],
           
             [[18.0,  1.1,    0.1],
             [19.0,  1.0,    0.0],
             [20.0,  -2.0,    0.0],
             [21.0,  1.0,    0.0],
             [22.0,  -1.0,    0.0],
             [23.0,  0.0,    0.0]]]
           
        e =  [[22.0,  -1.0,    0.0],
             [23.0,  0.0,    0.0]]

        wires = map(self.sweeper.make_wire_from_point_list, p)
        edge = self.sweeper.make_edge_from_points(e[0], e[1])
        connected_edges = self.sweeper.get_connected_shapes(wires[1], edge, "edge")
        assert len(connected_edges)==5, "p[1] is connected to e"
        connected_edges = self.sweeper.get_connected_shapes(wires[0], edge, "edge")
        assert len(connected_edges)==0, "p[0] is not connected to e"
        compound = pickle.load( open( "./get_front_surface 001.tmp", "rb" ) )

    def test_get_wires_from_edges(self):    
        point_pairs=[  [[0.0,  0.0,    0.0], [1.0,  1.0,    0.0]],
                       [[1.0,  1.0,    0.0], [2.0,  -2.0,    0.0]],
                       [[2.0,  -2.0,    0.0], [3.0,  0.0,    0.0]],
                       [[3.0,  0.0,    0.0], [4.0,  1.0,    0.0]],
                       
                       [[4.1,  1.0,    0.0], [5.0,  -1.0,    0.0]],
                       [[5.0,  -1.0,    0.0], [6.0,  0.0,    0.0]],
                       [[6.0,  0.0,    0.0], [7.0,  -1.0,    0.0]],
                       [[7.0,  -1.0,    0.0], [8.0,  0.0,    0.0]],
                       
                       [[18.0,  1.1,    0.1], [19.0,  1.0,    0.0]],
                       [[19.0,  1.0,    0.0], [20.0,  -2.0,    0.0]],
                       [[20.0,  -2.0,    0.0], [21.0,  1.0,    0.0]],
                       [[21.0,  1.0,    0.0], [22.0,  -1.0,    0.0]]]
        w1_p = [[0.0,  0.0,    0.0], [1.0,  1.0,    0.0], [2.0,  -2.0,    0.0], [3.0,  0.0,    0.0], [4.0,  1.0,    0.0]]
        w2_p = [[4.1,  1.0,    0.0], [5.0,  -1.0,    0.0], [6.0,  0.0,    0.0], [7.0,  -1.0,    0.0], [8.0,  0.0,    0.0]]
        w2_p_e = [[4.1,  1.0,    0.0], [5.0,  -1.0,    0.0], [6.0,  0.0,    0.0], [7.0,  -1.0,    0.0], [8.0,  0.0,    1.0]]
         
        edges = map(self.sweeper.make_edge_from_points, point_pairs)
        wires = self.sweeper.get_wires_from_edges(edges)
        assert len(wires)==3, "expect 3 wire from these edges"
        w1 = self.sweeper.make_wire_from_point_list(w1_p)
        assert self.sweeper.is_equal_wire(w1, wires[0]), "check wire[0] content"
        w2 = self.sweeper.make_wire_from_point_list(w2_p)
        assert self.sweeper.is_equal_wire(w2, wires[1]), "check wire[1] content"
        w2_e = self.sweeper.make_wire_from_point_list(w2_p_e)
        assert not self.sweeper.is_equal_wire(w2_e, wires[1]), "check wire[1] content"
        
    def test_get_extension_point(self):
        p=[[0.0,0.0,0.0],[1.0,1.0,1.0]]
        p_e = self.sweeper.get_extension_point(p[0], p[1], 1.732)
        pprint.pprint(p_e)
        assert self.sweeper.is_equal_point(p_e, [2.0, 2.0, 2.0]), "extension point is at 2,2,2"
        
    def test_extend_wire(self):
        p=[[0.0, 0.0, 0.0],
           [1.0, 1.0, 1.0],
           [2.0, 1.0, 1.0],
           [3.0, 0.0, 0.0]]
        p_extended = [[-1.0, -1.0, -1.0],
                      [1.0, 1.0, 1.0],
                      [2.0, 1.0, 1.0],
                      [4.0, -1.0, -1.0]]
        setattr(self.sweeper, 'sweep_orientation', 'x')
        w = self.sweeper.make_wire_from_point_list(p)
        w_extended = self.sweeper.make_wire_from_point_list(p_extended)
        w_tested = self.sweeper.extend_wire(w, 1.732, "both")
        assert self.sweeper.is_equal_wire(w_extended, w_tested), "the two wires should match"
        
'''
    def extend_wire(self, wire, length, direction="both"):
    #get_extension_point(self, point0, point1, length):
    #def get_wires_from_edges(self, edges):
    #def get_connected_shapes(self, whole_shape, part_shape, shape_type, hash_shapes=None, topo_w=None, vertices=None, level=0):
    #def join_nearby_edges(self, wires):
    #def reduce_wire_edge(self, wire):
    #def get_ordered_vertices_from_wire(self, wire):
    #def make_edge_from_vertices(self, vertices):
    #def make_vertex_from_point(self, point):
    #def make_edge_from_points(self, points):
    #def make_wire_from_point_list(self, points):
    #def make_wire_from_vertices(self, vertices):
    #def make_wire_from_edges(self, edges):
    #def get_point_from_vertex(self, vertex):
    #def is_equal_point(self, p1, p2):
    #def is_equal_vertex(self, v1, v2):
    #def is_equal_edge(self, e1, e2):
    #def is_equal_wire(self, w1, w2):
    #def contain_vertex(self, shape, vertex):
    #def count_vertices(self, shape):
    #def count_edges(self, shape):
    #def contain_edge(self, shape, edge):
'''

if __name__ == "__main__":
    unittest.main()