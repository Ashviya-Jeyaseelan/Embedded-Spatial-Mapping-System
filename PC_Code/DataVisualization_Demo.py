import open3d as o3d
import numpy as np

# File path to your .xyz file 
file_path = "data_demo.xyz"
#file_path = "data_hallway.xyz"

# Reads the point cloud data from file path
point_cloud_data = o3d.io.read_point_cloud(file_path)

# Obtains points from the point cloud data by converting into a NumPy array
points = np.asarray(point_cloud_data.points)

# Iterates through points in array
connect = []
for i in range(len(points)):
    # Creates a line from the current point to the next points
    # (i + 1) % len(points): Connects first and last point
    connect.append([i, (i + 1) % len(points)])

# Iterates through points in array
for i in range(len(points)):
    # Creates line between the edges, connecting it between the current point
    # and point 32 positions away for visualization
    connect.append([i, (i + 32) % len(points)])

# Creates line segments object and initialized empty line_segments
line_segments = o3d.geometry.LineSet()
line_segments.points = o3d.utility.Vector3dVector(points)
line_segments.lines = o3d.utility.Vector2iVector(connect)

# Visualize the point cloud data by connecting line segments
o3d.visualization.draw_geometries([point_cloud_data, line_segments])