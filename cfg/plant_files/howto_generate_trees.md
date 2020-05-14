- Download binvox: https://www.patrickmin.com/binvox/
- Convert the wrl file to binvox with the following command:
  ```Shell
  ./binvox -e -bb -1 -1 -1 1 1 1 -d 100 -rotx MESH_FILE.wrl
  ```
  Choose bounding box (-bb) and voxel grid size (-d) to achieve the desired resolution. In this case, a box from (-1, -1, -1) to (1, 1, 1) with a grid size of 100 is specified, which leads to a resolution of 0.02 m. Make sure that the bounding box is large enough to contain the complete mesh. -rotx converts the y-up coordinate system (default for wrl files) to z-up (default for octomap).
- Convert the binvox file to a binary octree using binvox2bt (comes with octomap framework)
  ```Shell
  binvox2bt MESH_FILE.binvox
  ```