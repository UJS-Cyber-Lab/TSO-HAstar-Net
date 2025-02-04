# ❗Sample ROS-C++ code will be made available upon finalization in the COMPAG journal❗
# TSO-HA*-Net: A Hybrid Global Path Planner for the Inspection Vehicle Used in Caged Poultry Houses

<p align="Justify">
  TSO-HA*-Net is a hybrid global path planner that combines TSO-HA* with topological planning, which allows the inspection vehicle to continuously traverse a predetermined trackless route within each poultry house and conduct house-to-house inspections. Initially, the spatiotemporally optimized Hybrid A*(TSO-HA*) is employed as the lower-level planner to efficiently construct a semi-structured topological network by integrating predefined inspection rules into the global grid map of the poultry houses. Subsequently, the Dijkstra algorithm is adopted to plan a smooth inspection route that aligns with the starting and ending poses, conforming to the network. Our work provides valuable case studies and algorithmic insights for similar inspection task.
</p>


# 🌐Related papers
#### Our related papers are now accessible:[]()

# 💡Contribution
*  <p align="Justify">An improved Hybrid A* algorithm is proposed, which demonstrates shorter computational time, reduced re-source consumption, and a higher success rate when applied to high-resolution maps. This improvement is achieved through a new heuristic value calculation using the Distance Reference Tree and simplified occupancy grid templates for collision detection.</p>
*  <p align="Justify">A hybrid path planner called TSO-HA*-Net is proposed, integrating the improved HA* algorithm, specific in-spection rules and topological planning. This planner is efficient and flexible for planning inspection routes in large-scale maps, establishing an intermediate planning scheme between point-to-point and full-coverage path planning. It addresses the route requirements for coop inspection and adapts to the actual poultry house envi-ronment.</p>
*  <p align="Justify">The rail-less navigation system based on the presented planning scheme offers a viable alternative to traditional electromagnetic and line-following navigation. This study provides valuable case studies and algorithmic refer-ences for inspection tasks in caged poultry houses and similar facilities. </p>

# ⭐Impact and Presentation
<p align="Justify">TSO-HA*-Net consists of two parts: the lower-level TSO-HA* module and the upper-level topological planning module. The overall flowchart is shown in Fig. 1.</p>

<p align="center">
<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Flowchart%20for%20planning%20inspection%20routes%20via%20TSO-HA-Net.png" width="80%" height="80%"> 
</p>

<p align="center">Fig. 1. Flowchart for planning inspection routes via TSO-HA*-Net.</p>

<p align="Justify">Connection points serve as entry and exit points for the inspection vehicle within the topological network. An illustrative example of initiating topological planning on a directed path of the semi-structured network is shown in Fig. 2, which illustrates how the planned path is seamlessly integrated into the network as an inspection route.</p>

<p align="center">
<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Merging%20the%20path%20into%20the%20network/a.png" width="50%" height="50%"> 
</p>
<p align="center">Fig. 2. Merging the path into the network.</p>

<p align="Justify">Planning inspection routes using TSO-HA*-Net demonstrates outstanding planning performance. In Fig. 3a, pathways through elongated inter-cage corridors maintain linear trajectories, while "U"-shaped intersections exhibit seamless connectivity and smooth transitions. This both helps mitigate tire wear and motor strain caused by stationary rotations in the differential-drive inspection vehicle and complies with the Ackermann steering geometry. By extract-ing key waypoints from linear path segments at regular intervals and storing them in a KD-tree format, detection points are designated where inspection vehicles can stop to identify diseased or dead poultry. Fig. 3b shows the route reaching specific positions, indicating that the inspection vehicle is able to access these points for charging or maintenance while complying with the network. The planned route enables house-to-house inspection, as shown in Fig. 4, reducing the need for additional vehicles. 
The TSO-HA*-Net algorithm efficiently reduces the computational time required for both single-house and mul-tiple-house route planning. Planning an inspection route for a single house takes an average of 257.98ms, while plan-ning the global route that involves cross-coop transfers requires only 546.62ms.</p>

| ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Single-house%20route%20planning/a.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Single-house%20route%20planning/b.png) |  
| :---: | :---: | 
| (a) | (b) | 

<p align="center">Fig. 3. Single-house route planning, (a) inspection route, (b) access to specific positions.</p>

<p align="center">
<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Multiple-house%20route%20planning/a.png" width="100%" height="100%"> 
</p>
<p align="center">Fig. 4. Multiple-house route planning.</p>

# ⚡Quick Start



# 📬Contact

If you have any questions, please feel free to contact: ***Yueping Sun*** ``sunypujs@ujs.edu.cn``

  

