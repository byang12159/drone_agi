import numpy as np
from stl import mesh
import plotly.graph_objects as go
from plotly.offline import download_plotlyjs, init_notebook_mode,  plot


def get_stl_color(x):
    #two "attribute byte count" bytes at the end of every triangle to store a 15-bit RGB color
    #bits 0 to 4 are the intensity level for blue (0 to 31)
    #bits 5 to 9 are the intensity level for green (0 to 31)
    #bits 10 to 14 are the intensity level for red (0 to 31)
    sb = f'{x:015b}'[::-1]
    r = str(int(255/31*int(sb[:5],base=2)))
    g = str(int(255/31*int(sb[5:10],base=2)))
    b = str(int(255/31*int(sb[10:15],base=2)))
    color = f'rgb({r},{g},{b})'
    return color


def stl2mesh3d(stl_file):
    stl_mesh = mesh.Mesh.from_file(stl_file)
    # stl_mesh is read by nympy-stl from a stl file; it is  an array of faces/triangles (i.e. three 3d points) 
    # this function extracts the unique vertices and the lists I, J, K to define a Plotly mesh3d
    p, q, r = stl_mesh.vectors.shape #(p, 3, 3)
    # the array stl_mesh.vectors.reshape(p*q, r) can contain multiple copies of the same vertex;
    # extract unique vertices from all mesh triangles
    vertices, ixr = np.unique(stl_mesh.vectors.reshape(p*q, r), return_inverse=True, axis=0)
    I = np.take(ixr, [3*k for k in range(p)])
    J = np.take(ixr, [3*k+1 for k in range(p)])
    K = np.take(ixr, [3*k+2 for k in range(p)])
    facecolor = np.vectorize(get_stl_color)(stl_mesh.attr.flatten())
    x,y,z = vertices.T
    trace = go.Mesh3d(x=x, y=y, z=z, i=I, j=J, k=K, facecolor=facecolor)
    # optional parameters to make it look nicer
    trace.update(flatshading=True)
    return trace


filename = 'chaser_drone_normal.stl'

trace = stl2mesh3d(filename)


layout = go.Layout(paper_bgcolor='rgb(220,220,220)',
            title_text='Pump house', title_x=0.5,
                    font_color='black',
            width=800,
            height=800,
            scene_camera=dict(eye=dict(x=1.25, y=-1.25, z=1)),
            scene_xaxis_visible=False,
            scene_yaxis_visible=False,
            scene_zaxis_visible=False)

fig = go.Figure(go.Mesh3d(trace),  layout=layout)

fig.data[0].update(lighting=dict(ambient= 0.18,
                                  diffuse= 1,
                                  fresnel=  .1,
                                  specular= 1,
                                  roughness= .1,
                                  facenormalsepsilon=0))
fig.data[0].update(lightposition=dict(x=3000,
                                      y=3000,
                                      z=10000))

plot(fig)
