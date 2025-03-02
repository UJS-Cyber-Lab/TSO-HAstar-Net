# <p align="Justify">TSO-HA*-Net: A Hybrid Global Path Planner for the Inspection Vehicles uesd in Poultry Houses</p>


Our work provides valuable case studies and algorithmic insights for similar inspection tasks.
<p align="Justify">
TSO-HA*-Net is a hybrid global path planner that combines TSO-HA* with topological planning, which allows the inspection vehicle to continuously traverse a predetermined trackless route within each poultry house and conduct house-to-house inspections. Initially, the spatiotemporally optimized Hybrid A*(TSO-HA*) is employed as the lower-level planner to efficiently construct a semi-structured topological network by integrating predefined inspection rules into the global grid map of the poultry houses. Subsequently, the Dijkstra's algorithm is adopted to plan a smooth inspection route that aligns with the starting and ending poses, conforming to the network.
</p>


# 🌐 Related Paper
#### Our related paper is now accessible in ***Agriculture-Basel*** Journal: [doi.org/10.3390/agriculture15050532](https://doi.org/10.3390/agriculture15050532)


# 💡 Contribution
*  <p align="Justify">An improved Hybrid A* algorithm is proposed, which demonstrates shorter computational time, reduced resource consumption, and a higher success rate when applied to high-resolution maps. This improvement is achieved through a new heuristic value calculation using the Distance Reference Tree and simplified occupancy grid templates for collision detection.</p>
*  <p align="Justify">A hybrid path planner called TSO-HA*-Net is proposed, integrating the improved HA* algorithm, specific inspection rules and topological planning. This planner is efficient and flexible for planning inspection routes in large-scale maps, establishing an intermediate planning scheme between point-to-point and full-coverage path planning. It addresses the route requirements for coop inspection and adapts to the actual poultry house environment.</p>
*  <p align="Justify">The rail-less navigation system based on the presented planning scheme offers a viable alternative to traditional electromagnetic and line-following navigation. This study provides valuable case studies and algorithmic references for inspection tasks in caged poultry houses and similar facilities. </p>


# ⭐ Impact and Presentation
<p align="Justify">TSO-HA*-Net consists of two parts: the lower-level TSO-HA* module and the upper-level topological planning module. Essentially, it performs the task of planning a topological network and determining a route within the network. The overall flowchart is shown in Fig.1.</p>

<p align="center">
<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Flowchart%20for%20planning%20inspection%20routes%20via%20TSO-HA-Net.png" width="80%" height="80%"> 
</p>

<p align="center">Fig.1. Flowchart for planning inspection routes via TSO-HA*-Net.</p>

<p align="Justify">Fig.2 demonstrates how the planned path is constrained by the topological network and relies on the connection points to merge seamlessly into the network. These connection points (indicated by labels 2 and 3) are key components in constructing the complete network, and serve as critical points for the inspection vehicle to enter and exit the network.</p>

<p align="center">
<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Merging%20the%20path%20into%20the%20network/a.png" width="40%" height="40%"> 
</p>
<p align="center">Fig.2. Merging the path into the network.</p>

<p align="Justify">Planning inspection routes using TSO-HA*-Net demonstrates outstanding planning performance. In Fig.3a, pathways through elongated inter-cage corridors maintain linear trajectories, while "U"-shaped intersections exhibit seamless connectivity and smooth transitions. This both helps mitigate tire wear and motor strain caused by stationary rotations in the differential-drive inspection vehicle and complies with the Ackermann steering geometry. By extracting key waypoints from linear path segments at regular intervals and storing them in a KD-tree format, detection points are designated where inspection vehicles can stop to identify diseased or dead poultry. Fig.3b shows the route reaching specific positions, indicating that the inspection vehicle is able to access these points for charging or maintenance while complying with the network. The planned route enables house-to-house inspection, as shown in Fig.4, reducing the need for additional vehicles. 
The TSO-HA*-Net algorithm efficiently reduces the computational time required for both single-house and mul-tiple-house route planning. Planning an inspection route for a single house takes an average of 257.98ms, while planning the global route that involves cross-coop transfers requires only 546.62ms.</p>

| <img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Single-house%20route%20planning/a.png" width="60%" height="60%"> | <img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Single-house%20route%20planning/b.png" width="60%" height="60%"> |  
| :---: | :---: | 
| (a) | (b) | 

<p align="center">Fig.3. Single-house topological planning. (a) Inspection route; (b) Access to specific positions.</p>

<p align="center">
<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Multiple-house%20route%20planning/a.png" width="100%" height="100%"> 
</p>
<p align="center">Fig.4. Multiple-house topological planning.</p>

<p align="Justify">TSO-HA*-Net has the potential to plan inspection routes in more intricate environments, as shown in the Fig.5 below, with computation times requiring only a few hundred milliseconds.</p>

<p align="center">
<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/A%20planning%20example%20in%20an%20intricate%20scenario.png" width="100%" height="100%"> 
</p>
<p align="center">Fig.5. A planning example in an intricate scenario.</p>


# ⚡ Quick Start
The tutorial will be released as soon as possible！


# 📬 Contact
If you have any questions, please feel free to contact: ***Yueping Sun*** ``sunypujs@ujs.edu.cn``    ***Cz Akria*** ``2222207069@stmail.ujs.edu.cn``

  

