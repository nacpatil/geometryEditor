from meshClass import *
from primitives import *
import numpy as np
from config import *
import json


# Create a grid of cuboids, with some overlapping to test collision
for i in range(3):
    for j in range(3):
        # Introduce overlap conditionally
        x_offset = 5 + i * 3
        y_offset = 8 + j * 3
        overlap_offset = 0 if (i + j) % 2 == 0 else -1  # Shift every other cube slightly

        # Call the cuboid function instead of direct instantiation
        cuboid(center=(x_offset + overlap_offset, y_offset + overlap_offset, 0), 
               size=(2.0, 2.0, 4.0), 
               title=f"3D Cuboid {i}-{j}")

        # Apply overlap offset to introduce collisions
        mesh_objects[-1].translate(dx=overlap_offset, dy=overlap_offset, dz=0)

# Display the 3D scene with collision detection
show_all_plots()

show_all_plots()

exit()
# Define an oriented cuboid with a custom basis

# Open and read the JSON file
inputData = None
with open('input.json', 'r') as file:
    inputData = json.load(file)

 
condenserLoc =  inputData[0]["translationVector"]

xC = inputData[1]["translationVector"]
xC[0]+= float(condenserLoc[0])
xC[1]+= float(condenserLoc[1])
xC[2]+= float(condenserLoc[2])

boxSizes = [  float(inputData[1]["attributes"]["length"]), float(inputData[1]["attributes"]["width"]), float(inputData[1]["attributes"]["height"]) ] 


print( boxSizes)

cuboid(center=(xC[0], xC[1], xC[2]), size=(boxSizes[0], boxSizes[1], boxSizes[2]), title="frontPanel")

fPanelVol =mesh_objects[-1].volume
print("frontPanel volume : ", fPanelVol)
'''
  {
    "name": "rightPanel",
    "attributes": {
      "length": 550,
      "width": 50,
      "height": 800
    },
    "translationVector": [ 650, 50, 50 ],
    "rotationVector": [ 0, 0, 90 ],
    "referencePoint": "condenser",
    "cadFile": "hx_panel.stl"
  }

'''

xC = inputData[2]["translationVector"]
xC[0]+= float(condenserLoc[0])
xC[1]+= float(condenserLoc[1])
xC[2]+= float(condenserLoc[2])

boxSizes = [  float(inputData[2]["attributes"]["length"]), float(inputData[2]["attributes"]["width"]), float(inputData[2]["attributes"]["height"]) ] 
rotationVector = inputData[2]["rotationVector"]

print(xC, boxSizes,rotationVector)
cuboid(center=(float(xC[0]), float(xC[1]), float(xC[2])), size=(boxSizes[0], boxSizes[1], boxSizes[2]), title="rightPanel")

mesh_objects[-1].rotate('z',90)
rightPanelVol =mesh_objects[-1].volume
print("rightPanel volume : ", rightPanelVol)
print("Total volume : ", rightPanelVol + fPanelVol)


show_all_plots()






exit()

star_polygon = PolygonSurface(
    x=[0, 1, 3, 1.5, 2, 0, -2, -1.5, -3, -1],
    y=[3, 1, 1, 0, -2, -1, -2, 0, 1, 1],
    z=0,
    title="Star Polygon"
)
star_polygon.translate(4,0,0)

obb_cuboid(
    center=(2, 3, 4), 
    size=(2.0, 1.0, 1.5), 
    u=(1, 1, 0),  # Diagonal in X-Y plane
    v=(-1, 1, 0), # Perpendicular to u
    w=(0, 0, 1)   # Stays aligned with Z
)



myPoly1 = PointLine(x=[0, 1, 2], y=[0, 1, 4], z=[0, 1, 8], title="Scatter 1")
myPoly2 = PointLine(x=[3, 4, 5], y=[9, 16, 25], z=[27, 64, 125], title="Scatter 2")
print("PointLine", myPoly1.x)


sphere(center=(0, 0, 0), radius=1.0, title="Sphere 1")
sphere(center=(3, 3, 3), radius=1.5, title="Sphere 2")
sphere(center=(-2, -2, 2), radius=5, title="Sphere 3")


for i in range(0,5):
    for j in range(0,5):
        cuboid(center=(5+i*3, 8+j*3, 0), size=(2.0, 2.0, 4.0), title="3D Cuboid")

mesh_objects[-1].rotate('x',45)

# Now plot everything in a **single unified plot**


show_all_plots()
