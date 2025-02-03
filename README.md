# TSO-HA*-Net: A Hybrid Global Path Planner for the Inspection Vehicle Used in Caged Poultry Houses
### ❗*Sample ROS-C++ code will be made available upon finalization in the COMPAG journal.*❗

TSO - HA* serves as a lower level planner and is employed to construct semi - structured topological networks. It integrates predefined inspection rules into the global grid map of the poultry house. This algorithm not only preserves the smoothness of mixed A * (HA *) paths but also reduces time and computational overhead, thus enhancing the speed and efficiency of network generation. Based on the topology network constructed by TSO - HA *, topology planning aims to plan a smooth inspection route that conforms to the starting and ending postures. By using Dijkstra's algorithm on the basis of this semi - structured topology network, the shortest path that meets the conditions can be found, ensuring that inspection vehicles can conduct inspections along this route.


# 🌐Related papers
#### Our related papers are now accessible:[]()

# ⚡Quick Start


# ⭐Impact and Presentation
In order to clearly present the logical relationship and dynamic evolution process between various elements, a flowchart is used as a visualization tool, which can present complex processes in a concise and intuitive way.

<p align="center">
<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Flowchart%20for%20planning%20inspection%20routes%20via%20TSO-HA-Net.png" width="80%" height="80%"> 
</p>

<p align="center">Fig. 1. Flowchart for planning inspection routes via TSO-HA*-Net.</p>

Connection points serve as entry and exit points for the inspection vehicle within the topological network. An il-lustrative example of initiating topological planning on a directed path of the semi-structured network is shown in Fig. 2, which illustrates how the planned path is seamlessly integrated into the network as an inspection route.

<p align="center">
<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Merging%20the%20path%20into%20the%20network/a.png" width="50%" height="50%"> 
</p>
<p align="center">Fig. 2. Merging the path into the network.</p>

Planning inspection routes using TSO-HA*-Net demonstrates outstanding planning performance. In Fig. 3a, pathways through elongated inter-cage corridors maintain linear trajectories, while "U"-shaped intersections exhibit seamless connectivity and smooth transitions. This both helps mitigate tire wear and motor strain caused by stationary rotations in the differential-drive inspection vehicle and complies with the Ackermann steering geometry. By extract-ing key waypoints from linear path segments at regular intervals and storing them in a KD-tree format, detection points are designated where inspection vehicles can stop to identify diseased or dead poultry. Fig. 3b shows the route reaching specific positions, indicating that the inspection vehicle is able to access these points for charging or maintenance while complying with the network. The planned route enables house-to-house inspection, as shown in Fig. 4, reducing the need for additional vehicles. 
The TSO-HA*-Net algorithm efficiently reduces the computational time required for both single-house and mul-tiple-house route planning. Planning an inspection route for a single house takes an average of 257.98ms, while plan-ning the global route that involves cross-coop transfers requires only 546.62ms.

| ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Single-house%20route%20planning/a.png) | ![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Single-house%20route%20planning/b.png) |  
| :---: | :---: | 
| (a) | (b) | 

<p align="center">Fig. 3. Single-house route planning, (a) inspection route, (b) access to specific positions.</p>

<p align="center">
<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Multiple-house%20route%20planning/a.png" width="100%" height="100%"> 
</p>
<p align="center">Fig. 4. Multiple-house route planning.</p>

# 💡Contribution
### The main contributions of this study are as follows:
*  An improved Hybrid A* algorithm is proposed, which demonstrates shorter computational time, reduced re-source consumption, and a higher success rate when applied to high-resolution maps. This improvement is achieved through a new heuristic value calculation using the Distance Reference Tree and simplified occupancy grid templates for collision detection.
*  A hybrid path planner called TSO-HA*-Net is proposed, integrating the improved HA* algorithm, specific in-spection rules and topological planning. This planner is efficient and flexible for planning inspection routes in large-scale maps, establishing an intermediate planning scheme between point-to-point and full-coverage path planning. It addresses the route requirements for coop inspection and adapts to the actual poultry house envi-ronment.
*  The rail-less navigation system based on the presented planning scheme offers a viable alternative to traditional electromagnetic and line-following navigation. This study provides valuable case studies and algorithmic refer-ences for inspection tasks in caged poultry houses and similar facilities. 


# 📬Contact

  If you have any questions, please feel free to contact: ***Yueping Sun*** ``sunypujs@ujs.edu.cn``

