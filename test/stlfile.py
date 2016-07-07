import os
from OCC.StlAPI import StlAPI_Writer
from OCC.BRepPrimAPI import BRepPrimAPI_MakeBox

my_box = BRepPrimAPI_MakeBox(10., 20., 30.).Shape()

# set the directory where to output the
directory = os.path.split(__name__)[0]
stl_output_dir = os.path.abspath(os.path.join(directory, "models"))

# make sure the path exists otherwise OCE get confused
assert os.path.isdir(stl_output_dir)
stl_output_file = os.path.join(stl_output_dir, "box.stl")

# its advisable to write binary STL files, this result in a
# file size reduction of ~10*
stl_ascii_format = False

stl_export = StlAPI_Writer()
stl_export.Write(my_box, stl_output_file, stl_ascii_format)
