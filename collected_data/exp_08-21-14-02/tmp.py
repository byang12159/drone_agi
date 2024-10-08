
import pyvista as pv
import numpy as np

# Define the start and end points
start_point = np.array([0, 0, 0])
end_point = np.array([1, 1, 0])

# Create a function to generate dashed lines
def create_dashed_line(start, end, num_dashes=10, dash_ratio=0.5):
    # Calculate the direction vector and the length of each dash and gap
    direction = end - start
    total_length = np.linalg.norm(direction)
    dash_length = (total_length / num_dashes) * dash_ratio
    gap_length = (total_length / num_dashes) * (1 - dash_ratio)
    
    # Normalize direction vector
    direction = direction / np.linalg.norm(direction)
    
    # Generate points for the dashed line
    points = []
    for i in range(num_dashes):
        dash_start = start + direction * (i * (dash_length + gap_length))
        dash_end = dash_start + direction * dash_length
        points.append((dash_start, dash_end))
    
    return points

# Create dashed line segments
dashed_segments = create_dashed_line(start_point, end_point)

# Plot the dashed line using PyVista
plotter = pv.Plotter()
for seg_start, seg_end in dashed_segments:
    line = pv.Line(seg_start, seg_end)
    plotter.add_mesh(line, color='black', line_width=2)

plotter.show()