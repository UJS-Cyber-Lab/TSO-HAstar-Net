# TSO-HA*-Net

<p align="Justify">
TSO-HA*-Net is a hybrid global path planner for the inspection vehicles used in caged poultry houses that combines TSO-HA* with topological planning, which allows the inspection vehicle to continuously traverse a predetermined trackless route within each poultry house and conduct house-to-house inspections. Initially, the spatiotemporally optimized Hybrid A*(TSO-HA*) is employed as the lower-level planner to efficiently construct a semi-structured topological network by integrating predefined inspection rules into the global grid map of the poultry houses. Subsequently, the Dijkstra's algorithm is adopted to plan a smooth inspection route that aligns with the starting and ending poses, conforming to the network. TSO-HA*-Net provides valuable case studies and algorithmic insights for similar inspection tasks.
</p>

# 🌐 Related paper
Our related paper is now accessible in ***Agriculture-Basel*** : [doi.org/10.3390/agriculture15050532](https://doi.org/10.3390/agriculture15050532).

# 💡 Contribution
*  <p align="Justify">An improved Hybrid A* algorithm is proposed, which demonstrates shorter computational time, reduced resource consumption, and a higher success rate when applied to high-resolution maps. This improvement is achieved through a new heuristic value calculation using the Distance Reference Tree and simplified occupancy grid templates for collision detection.</p>
*  <p align="Justify">A hybrid path planner called TSO-HA*-Net is proposed, integrating the improved HA* algorithm, specific inspection rules and topological planning. This planner is efficient and flexible for planning inspection routes in large-scale maps, establishing an intermediate planning scheme between point-to-point and full-coverage path planning. It addresses the route requirements for coop inspection and adapts to the actual poultry house environment.</p>
*  <p align="Justify">The rail-less navigation system based on the presented planning scheme offers a viable alternative to traditional electromagnetic and line-following navigation. This study provides valuable case studies and algorithmic references for inspection tasks in caged poultry houses and similar facilities. </p>

# ⭐ Impact and Presentation
<p align="Justify">TSO-HA*-Net consists of two parts: the lower-level TSO-HA* module and the upper-level topological planning module. Essentially, it performs the task of planning a topological network and determining a route within the network.</p>
<p align="center">
<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/pic/Flowchart%20for%20planning%20inspection%20routes%20via%20TSO-HA-Net.png" width="60%" height="60%"> 
</p>
<p align="center">Fig.1. Flowchart for planning inspection routes via TSO-HA*-Net.</p>

<p align="Justify">A semi-structured topological network is constructed using the TSO-HA* algorithm, as shown in Fig.2a. Subsequently, topological planning is conducted based on this network. Fig.2b demonstrates how the planned path is constrained by the topological network and relies on the connection points to merge seamlessly into the network.</p>

| <img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/pic/Semi-structured%20topological%20network/a.png" width="70%" height="70%"> | <img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/pic/Semi-structured%20topological%20network/b.png" width="70%" height="70%"> |   
| :---: | :---: |  
| (a) | (b) |  

<p align="center">Fig.2. Semi-structured topological network. (a) Network construction; (b) Merging the path into the network.</p>


<p align="Justify">In Fig.3a, pathways through elongated inter-cage corridors maintain linear trajectories, while “U”-shaped intersections exhibit seamless connectivity and smooth transitions. Fig.3b illustrates that the inspection route retains the capability to cover specific areas, indicating that the inspection vehicle can reach designated locations via the shortest path while adhering to the constraints of the topological network. The planned route enables house-to-house inspection, as shown in Fig.4. </p>

| <img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/pic/Single-house%20route%20planning/a.png" width="60%" height="60%"> | <img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/pic/Single-house%20route%20planning/b.png" width="60%" height="60%"> |   
| :---: | :---: |  
| (a) | (b) | 

<p align="center">Fig.3. Single-house topological planning. (a) Inspection route; (b) Access to specific positions.</p>

<p align="center">
<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/pic/Multiple-house%20route%20planning/a.png" width="90%" height="100%"> 
</p>
<p align="center">Fig.4. Multiple-house topological planning.</p>

<p align="Justify">TSO-HA*-Net has the potential to plan inspection routes in more intricate environments, as shown in the Fig.5.</p>
<p align="center">
<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/pic/A%20planning%20example%20in%20an%20intricate%20scenario.png" width="90%" height="100%"> 
</p>
<p align="center">Fig.5. A planning example in an intricate scenario.</p>

# ⚡ Quick Start
```shell
cd ~
git clone https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net.git
cd TSO-HAstar-Net
catkin_make
source devel/setup.bash
```
*  TSO-HA*-Net:
```shell 
roslaunch tso_hastar_net inspection.launch
rosrun rqt_service_caller rqt_service_caller    # Service: /inspection_task  see /src/tso_hastar_net/srv/chibot_task.srv for details
```
*  TSO-HA*:
```shell
roslaunch tso_hastar_net simple_astar.launch
```

# 🌴 Prerequisites
#### TSO-HA*-Net is tested in Ubuntu 20.04. Please install the following libraries before compilation.
**1. ROS (melodic or noetic)**

**2. eigen: sudo apt-get install libeigen3-dev**

**3. pcl: sudo apt-get install libpcl-dev**

**4. ompl**

# 📬 Contact
If you have any questions, please feel free to contact: 

***Yueping Sun*** ``sunypujs@ujs.edu.cn``   

***Cz Akria*** ``2222207069@stmail.ujs.edu.cn``


  

