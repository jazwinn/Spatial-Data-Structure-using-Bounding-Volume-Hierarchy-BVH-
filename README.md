# Bouding Volume Hierachy

Demonstration of a Bounding Volume Hierarchy (BVH) for spatial partitioning and acceleration of collision detection or ray tracing.


## Demo

![1758029480062](https://github.com/user-attachments/assets/9491e279-389a-4a65-a2ce-aa4b0ffbf0f4)

## Overview

A **Bounding Volume Hierarchy** is a tree structure on a set of geometric objects. Each node contains a **bounding volume** that encloses a subset of the objects. BVHs are widely used in:

- **Ray tracing** for fast intersection tests.
- **Collision detection** in physics engines.
- **Visibility determination** and scene culling.

The main idea is to **reduce the number of intersection tests** by testing bounding volumes before testing the actual objects inside them.

### Prerequisites
Before you begin, make sure you have the following installed:
- **Cmake**
- **Cmake with visual studio**
- **vcpkg**

## Install Packages via vcpkg

**Windows (x64):**

```powershell
.\vcpkg\vcpkg install glm:x64-windows
.\vcpkg\vcpkg install glad:x64-windows
.\vcpkg\vcpkg install glfw3:x64-windows
.\vcpkg\vcpkg install lodepng:x64-windows
.\vcpkg\vcpkg install gtest:x64-windows
.\vcpkg\vcpkg install fmt:x64-windows
.\vcpkg\vcpkg install imgui[glfw-binding,opengl3-binding]:x64-windows
.\vcpkg\vcpkg install imguizmo:x64-windows
.\vcpkg\vcpkg install stb:x64-windows
```
  
**Linux (x64):**
```bash
   ./vcpkg/vcpkg install glm
   ./vcpkg/vcpkg install glad
   ./vcpkg/vcpkg install glfw3
   ./vcpkg/vcpkg install lodepng
   ./vcpkg/vcpkg install gtest
   ./vcpkg/vcpkg install fmt
   ./vcpkg/vcpkg install imgui[glfw-binding,opengl3-binding]
   ./vcpkg/vcpkg install imguizmo
   ./vcpkg/vcpkg install stb
```

## Steps

1. **Clone the Repository**

```bash
git clone https://github.com/<your-username>/Spatial-Data-Structure-using-Bounding-Volume-Hierarchy-BVH-.git
cd Spatial-Data-Structure-using-Bounding-Volume-Hierarchy-BVH-
```

2. **Create a Build Directory**

```bash
mkdir build
cd build
```

3. **Configure the Project with CMake**

```bash
cmake .. -DCMAKE_TOOLCHAIN_FILE=../vcpkg/scripts/buildsystems/vcpkg.cmake
```

4. **Build the Project**

* **Windows (Visual Studio)**

```powershell
cmake --build . --config Release
```

* **Linux**

```bash
cmake --build .
```

5. **Run the Executable**

```bash
./cs350-demo.exe
```


## Side Note
MODEL ASSETS REMOVED FOR COPYRIGHT REASON, PM FOR MORE DETAIL
You can see visual representation of a bounding volume hierachy under "BVH project\Visual Graph"


Graphs .dot or .txt : "\cs350su25_jazwinn.ng_a3\Submission Resource\dot&txt"
Graphs .png : "\cs350su25_jazwinn.ng_a3\Submission Resource\png"
Graphs .svg : "\cs350su25_jazwinn.ng_a3\Submission Resource\svg"

Switch Camera/View Frustrum Camera: 
1. Imgui Panel "BVH"
2. Header "Camera"
3. Select "Switch to Frustrum" to true to control the frustum, 
   switch back (false) to gain control of the other camera and see the frustum.
   Select "Draw Frustrum" to turn drawing frustrum on and off.

   
