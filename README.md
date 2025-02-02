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

<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_project/blob/main/picture/%E8%AE%BA%E6%96%87%E4%B8%AD%E6%B6%89%E5%8F%8A%E7%9A%84%E5%9B%BE%E7%89%87/1.%E4%BB%8B%E7%BB%8D/%E5%A4%8D%E6%9D%82%E9%B8%A1%E8%88%8D%E7%8E%AF%E5%A2%83.png" width="100%" height="100%"> 

<p align="center">Fig. 1. The site structure of a large-scale caged poultry farm.</p>

## 2.2 Problem description
  The path planning of poultry house inspection vehicles needs to take into account requirements such as obstacle avoidance, maintaining straight driving, and smooth paths. The traditional HA algorithm has problems such as slow planning speed and high resource consumption. This study proposes the TSO-HA algorithm and TSO-HA * - Net hybrid path planner to meet the requirements.
## 2.3 Overall framework
  TSO-HA*-Net consists of the underlying TSO-HA algorithm and the upper layer topology planning. Firstly, determine the topology nodes and connections, use TSO-HA algorithm for path pre planning, construct a semi-structured topology network and cost matrix, and then use Dijkstra algorithm to plan the complete inspection route.

<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_project/blob/main/picture/%E6%B5%81%E7%A8%8B%E5%9B%BE%E4%B8%AD%E6%B6%89%E5%8F%8A%E7%9A%84%E5%9B%BE%E7%89%87/%E6%B5%81%E7%A8%8B%E5%9B%BE.png" width="100%" height="100%"> 

<p align="center">Fig. 2. Flowchart for planning inspection routes via TSO-HA*-Net.</p>

## 2.4 TSO-HA Bottom level path planner
  Improved HA * algorithm, using specific node expansion strategy, distance reference tree (DRT), and simplified occupancy grid template to enhance computational efficiency, used for building semi-structured topology networks.

| ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_project/blob/main/picture/%E6%B5%81%E7%A8%8B%E5%9B%BE%E4%B8%AD%E6%B6%89%E5%8F%8A%E7%9A%84%E5%9B%BE%E7%89%87/DRT%E7%9A%84h%E8%AE%A1%E7%AE%97/a.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_project/blob/main/picture/%E6%B5%81%E7%A8%8B%E5%9B%BE%E4%B8%AD%E6%B6%89%E5%8F%8A%E7%9A%84%E5%9B%BE%E7%89%87/DRT%E7%9A%84h%E8%AE%A1%E7%AE%97/b.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_project/blob/main/picture/%E6%B5%81%E7%A8%8B%E5%9B%BE%E4%B8%AD%E6%B6%89%E5%8F%8A%E7%9A%84%E5%9B%BE%E7%89%87/DRT%E7%9A%84h%E8%AE%A1%E7%AE%97/c.png) |  
| :---: | :---: | :---: |  
| (a) | (b) | (c) |  
| ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_project/blob/main/picture/%E6%B5%81%E7%A8%8B%E5%9B%BE%E4%B8%AD%E6%B6%89%E5%8F%8A%E7%9A%84%E5%9B%BE%E7%89%87/DRT%E7%9A%84h%E8%AE%A1%E7%AE%97/d.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_project/blob/main/picture/%E6%B5%81%E7%A8%8B%E5%9B%BE%E4%B8%AD%E6%B6%89%E5%8F%8A%E7%9A%84%E5%9B%BE%E7%89%87/DRT%E7%9A%84h%E8%AE%A1%E7%AE%97/e.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_project/blob/main/picture/%E6%B5%81%E7%A8%8B%E5%9B%BE%E4%B8%AD%E6%B6%89%E5%8F%8A%E7%9A%84%E5%9B%BE%E7%89%87/DRT%E7%9A%84h%E8%AE%A1%E7%AE%97/f.png) |  
| (d) | (e) | (f) |  

<p align="center">Fig. 3. DRT-based  calculation, (a) eight-neighbor node expansion in A*, (b) turning point extraction, (c) edge construction, (d) approximation of shortest piecewise-linear A* path, (e) interpolation of piecewise-linear A*path and path distance calculation, (f) heuristic value computation.</p>

  
## 2.5 Upper level topology planning
  Introduce connection points to enable vehicles to smoothly enter and exit the network, manage path points using Hash Maps, construct a cost matrix based on TSO-HA * path length, and use Dijkstra's algorithm to search for the shortest path in the network to determine the inspection route.

| ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_project/blob/main/picture/%E6%B5%81%E7%A8%8B%E5%9B%BE%E4%B8%AD%E6%B6%89%E5%8F%8A%E7%9A%84%E5%9B%BE%E7%89%87/%E4%B8%8A%E5%B1%82%E6%8B%93%E6%89%91%E8%A7%84%E5%88%92/a.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_project/blob/main/picture/%E6%B5%81%E7%A8%8B%E5%9B%BE%E4%B8%AD%E6%B6%89%E5%8F%8A%E7%9A%84%E5%9B%BE%E7%89%87/%E4%B8%8A%E5%B1%82%E6%8B%93%E6%89%91%E8%A7%84%E5%88%92/b.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_project/blob/main/picture/%E6%B5%81%E7%A8%8B%E5%9B%BE%E4%B8%AD%E6%B6%89%E5%8F%8A%E7%9A%84%E5%9B%BE%E7%89%87/%E4%B8%8A%E5%B1%82%E6%8B%93%E6%89%91%E8%A7%84%E5%88%92/c.png) |  
| :---: | :---: | :---: |  
| (a) | (b) | (c) |  

<p align="center">Fig. 4. Diagram of connection points, (a) path planning starting from the starting point to   and the corresponding set  , (b) path planning starting from   to the ending point and the corresponding set  , (c) set   between the connection points.</p>

# 3.Experiment and data analysis
  Conduct comparative experiments based on the Jetson Orin NX platform to verify algorithm performance.
## 3.1 Line collision detection
  The improved occupancy grid template is more efficient than the OBB collision detection algorithm, reducing front-end planning time, and pre collision detection can avoid planning failures and improve planning efficiency.

| ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_project/blob/main/picture/%E8%AE%BA%E6%96%87%E4%B8%AD%E6%B6%89%E5%8F%8A%E7%9A%84%E5%9B%BE%E7%89%87/4.%E6%A3%80%E6%B5%8B%E7%A2%B0%E6%92%9E%E7%9A%84%E8%A7%84%E5%88%92%E6%95%88%E6%9E%9C%E5%9B%BE/%E9%99%B7%E5%85%A5%E7%AA%84%E9%81%93.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_project/blob/main/picture/%E8%AE%BA%E6%96%87%E4%B8%AD%E6%B6%89%E5%8F%8A%E7%9A%84%E5%9B%BE%E7%89%87/4.%E6%A3%80%E6%B5%8B%E7%A2%B0%E6%92%9E%E7%9A%84%E8%A7%84%E5%88%92%E6%95%88%E6%9E%9C%E5%9B%BE/%E4%B8%8D%E9%99%B7%E5%85%A5%E7%AA%84%E9%81%93.png) |  
| :---: | :---: | 
| (a) | (b) | 
 

<p align="center">Fig. 5. Planning effects with and without prior collision detection, (a) planning gets stuck in the narrow passage, (b) planning escapes from the narrow passage.</p>

## 3.2 Inspection route planning based on TSO-HA Net
  The inspection route planning based on TSO-HA Net has excellent performance, which can achieve in house and inter house inspections, reduce vehicle demand, and has short calculation time. It can also support multi vehicle collaborative inspections.

| ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_project/blob/main/picture/%E8%AE%BA%E6%96%87%E4%B8%AD%E6%B6%89%E5%8F%8A%E7%9A%84%E5%9B%BE%E7%89%87/6.%E5%A4%9A%E6%9C%BA%E5%8D%8F%E5%90%8C%E8%A7%84%E5%88%92/a.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_project/blob/main/picture/%E8%AE%BA%E6%96%87%E4%B8%AD%E6%B6%89%E5%8F%8A%E7%9A%84%E5%9B%BE%E7%89%87/6.%E5%A4%9A%E6%9C%BA%E5%8D%8F%E5%90%8C%E8%A7%84%E5%88%92/b.png) |  
| :---: | :---: | 
| (a) | (b) | 

<p align="center">Fig. 6. Single-house route planning, (a) inspection route, (b) access to specific positions.</p>

## 3.3 H2 Calculation
  The TSO-HA algorithm using DRT performs excellently in terms of planning success rate, computation time, and resource consumption, with significant advantages over LDP-MAP and A-dis.
# 4.Conclusion
  The combination of TSO-HA * - Net and topology planning shows high efficiency in poultry house experiments. The TSO-HA algorithm significantly reduces computation time and resource consumption, and TSO-HA * - Net only takes 546.62ms to plan the global route, which can meet the needs of poultry house inspection and replace traditional navigation methods. In the future, it will integrate positioning and navigation capabilities to achieve more accurate execution of inspection tasks.



# 📬Contact

  If you have any questions, please feel free to contact: **Yueping Sun** ``sunypujs@ujs.edu.cn``

