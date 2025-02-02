# TSO-HA*-Net
### TSO-HA*-Net: A Hybrid Global Path Planner for the Inspection Vehi-cle Used in Caged Poultry Houses
*Sample ROS-C++ code will be made available upon finalization in the COMPAG journal.*

# Related papers
Our related papers are now accessible:[TSO-HA*-Net: A Hybrid Global Path Planner for the Inspection Vehi-cle Used in Caged Poultry Houses]()

# 1.Research background and purpose
The scale of poultry farming has expanded, and the application of intelligent devices has increased. However, manual inspection of cage poultry houses is difficult, and inspection vehicles are urgently needed. The traditional track inspection scheme has problems such as easy damage to the track and tedious maintenance. The existing trackless inspection scheme lacks a stable and reliable path planner, and the "one room, one vehicle" mode has high costs. This study aims to propose a new global path planner to address the aforementioned issues.

# 2.Methods
## 2.1 Environmental analysis and mapping
  Represent the poultry house environment as a global occupancy grid map, and build a semi-structured topological network based on it to provide a foundation for path planning.

<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/The%20site%20structure%20of%20a%20large-scale%20caged%20poultry%20farm.png" width="100%" height="100%"> 

<p align="center">Fig. 1. The site structure of a large-scale caged poultry farm.</p>

## 2.2 Problem description
  The path planning of poultry house inspection vehicles needs to take into account requirements such as obstacle avoidance, maintaining straight driving, and smooth paths. The traditional HA algorithm has problems such as slow planning speed and high resource consumption. This study proposes the TSO-HA algorithm and TSO-HA * - Net hybrid path planner to meet the requirements.
## 2.3 Overall framework
  TSO-HA*-Net consists of the underlying TSO-HA algorithm and the upper layer topology planning. Firstly, determine the topology nodes and connections, use TSO-HA algorithm for path pre planning, construct a semi-structured topology network and cost matrix, and then use Dijkstra algorithm to plan the complete inspection route.

<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Flowchart%20for%20planning%20inspection%20routes%20via%20TSO-HA-Net.png" width="100%" height="100%"> 

<p align="center">Fig. 2. Flowchart for planning inspection routes via TSO-HA*-Net.</p>

## 2.4 TSO-HA Bottom level path planner
  Improved HA * algorithm, using specific node expansion strategy, distance reference tree (DRT), and simplified occupancy grid template to enhance computational efficiency, used for building semi-structured topology networks.

| ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/DRT-based%20calculation/a.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/DRT-based%20calculation/b.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/DRT-based%20calculation/c.png) |  
| :---: | :---: | :---: |  
| (a) | (b) | (c) |  
| ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/DRT-based%20calculation/d.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/DRT-based%20calculation/e.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/DRT-based%20calculation/f.png) |  
| (d) | (e) | (f) |  

<p align="center">Fig. 3. DRT-based  calculation, (a) eight-neighbor node expansion in A*, (b) turning point extraction, (c) edge construction, (d) approximation of shortest piecewise-linear A* path, (e) interpolation of piecewise-linear A*path and path distance calculation, (f) heuristic value computation.</p>

  
## 2.5 Upper level topology planning
  Introduce connection points to enable vehicles to smoothly enter and exit the network, manage path points using Hash Maps, construct a cost matrix based on TSO-HA * path length, and use Dijkstra's algorithm to search for the shortest path in the network to determine the inspection route.

| ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Diagram%20of%20connection%20points/a.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Diagram%20of%20connection%20points/b.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Diagram%20of%20connection%20points/c.png) |  
| :---: | :---: | :---: |  
| (a) | (b) | (c) |  

<p align="center">Fig. 4. Diagram of connection points, (a) path planning starting from the starting point to   and the corresponding set  , (b) path planning starting from   to the ending point and the corresponding set  , (c) set   between the connection points.</p>

# 3.Experiment and data analysis
  Conduct comparative experiments based on the Jetson Orin NX platform to verify algorithm performance.
## 3.1 Line collision detection
  The improved occupancy grid template is more efficient than the OBB collision detection algorithm, reducing front-end planning time, and pre collision detection can avoid planning failures and improve planning efficiency.

| ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Planning%20effects%20with%20and%20without%20prior%20collision%20detection/a.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Planning%20effects%20with%20and%20without%20prior%20collision%20detection/b.png) |  
| :---: | :---: | 
| (a) | (b) | 
 

<p align="center">Fig. 5. Planning effects with and without prior collision detection, (a) planning gets stuck in the narrow passage, (b) planning escapes from the narrow passage.</p>

## 3.2 Inspection route planning based on TSO-HA Net
  The inspection route planning based on TSO-HA Net has excellent performance, which can achieve in house and inter house inspections, reduce vehicle demand, and has short calculation time. It can also support multi vehicle collaborative inspections.

| ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Single-house%20route%20planning/a.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Single-house%20route%20planning/b.png) |  
| :---: | :---: | 
| (a) | (b) | 

<p align="center">Fig. 6. Single-house route planning, (a) inspection route, (b) access to specific positions.</p>

## 3.3 H2 Calculation
  The TSO-HA algorithm using DRT performs excellently in terms of planning success rate, computation time, and resource consumption, with significant advantages over LDP-MAP and A-dis.
# 4.Conclusion
  The combination of TSO-HA * - Net and topology planning shows high efficiency in poultry house experiments. The TSO-HA algorithm significantly reduces computation time and resource consumption, and TSO-HA * - Net only takes 546.62ms to plan the global route, which can meet the needs of poultry house inspection and replace traditional navigation methods. In the future, it will integrate positioning and navigation capabilities to achieve more accurate execution of inspection tasks.



# 📬Contact

  If you have any questions, please feel free to contact: **Yueping Sun** ``sunypujs@ujs.edu.cn``

