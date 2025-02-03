# TSO-HA*-Net: A Hybrid Global Path Planner for the Inspection Vehicle Used in Caged Poultry Houses
### *Sample ROS-C++ code will be made available upon finalization in the COMPAG journal.*

# 🌐Related papers
#### Our related papers are now accessible:[TSO-HA*-Net: A Hybrid Global Path Planner for the Inspection Vehicle Used in Caged Poultry Houses]()

# ⚡Quick Start
*  ***TSO - HA***
> As a lower level planner, it is used to construct semi-structured topological networks. Integrate predefined inspection rules into the global grid map of the poultry house. This algorithm preserves the smoothness of mixed A * (HA *) paths while reducing time and computational overhead, improving the speed and efficiency of network generation.
*  ***Topology planning***
> Based on the topology network constructed by TSO-HA *, plan a smooth inspection route that conforms to the starting and ending postures. On the basis of the semi-structured topology network constructed by TSO-HA *, Dijkstra's algorithm can find the shortest path that meets the conditions, ensuring that inspection vehicles can conduct inspections along this route.


# ⭐Impact and Presentation
In order to clearly present the logical relationship and dynamic evolution process between various elements, a flowchart is used as a visualization tool, which can present complex processes in a concise and intuitive way.

<p align="center">
<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Flowchart%20for%20planning%20inspection%20routes%20via%20TSO-HA-Net.png" width="80%" height="80%"> 
</p>

<p align="center">Fig. 1. Flowchart for planning inspection routes via TSO-HA*-Net.</p>

Connection points serve as entry and exit points for the inspection vehicle within the topological network. An il-lustrative example of initiating topological planning on a directed path of the semi-structured network is shown in Fig. 2, which illustrates how the planned path is seamlessly integrated into the network as an inspection route.

<p align="center">
<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net/blob/main/picture/Merging%20the%20path%20into%20the%20network/a.png" width="100%" height="100%"> 
</p>
<p align="center">Fig. 2. Merging the path into the network.</p>

# 💡Contribution
### The main contributions of this study are as follows:
*  An improved Hybrid A* algorithm is proposed, which demonstrates shorter computational time, reduced re-source consumption, and a higher success rate when applied to high-resolution maps. This improvement is achieved through a new heuristic value calculation using the Distance Reference Tree and simplified occupancy grid templates for collision detection.
*  A hybrid path planner called TSO-HA*-Net is proposed, integrating the improved HA* algorithm, specific in-spection rules and topological planning. This planner is efficient and flexible for planning inspection routes in large-scale maps, establishing an intermediate planning scheme between point-to-point and full-coverage path planning. It addresses the route requirements for coop inspection and adapts to the actual poultry house envi-ronment.
*  The rail-less navigation system based on the presented planning scheme offers a viable alternative to traditional electromagnetic and line-following navigation. This study provides valuable case studies and algorithmic refer-ences for inspection tasks in caged poultry houses and similar facilities. 


# 📬Contact

  If you have any questions, please feel free to contact: ***Yueping Sun*** ``sunypujs@ujs.edu.cn``

