# URDFImport
3D Slicer scripted module to import URDF files. Creates kinematic digital twin representations of robots, including joint functionality and movement limits.

Extended from [https://gist.github.com/lassoan/e174e853cd78ad93a1cd54b32debdac8](https://gist.github.com/lassoan/e174e853cd78ad93a1cd54b32debdac8) with additions of prismatic joints, corrected positioning for 3D meshes, and joint limits.

Rotation representation conversion from https://github.com/li-xl/rotationconverter/tree/master and https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/ (converted to python).

# Future Directions
Finish addition of xacro to urdf converter,
add rotation and translation selection sliders in module for more accuracy
