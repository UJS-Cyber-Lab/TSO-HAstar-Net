# TSO-HA*-Net
### TSO-HA*-Net: A Hybrid Global Path Planner for the Inspection Vehi-cle Used in Caged Poultry Houses
*Sample ROS-C++ code will be made available upon finalization in the COMPAG journal.*

# 1.Research background and purpose
  The scale of poultry farming has expanded, and the application of intelligent devices has increased. However, manual inspection of cage poultry houses is difficult, and inspection vehicles are urgently needed. The traditional track inspection scheme has problems such as easy damage to the track and tedious maintenance. The existing trackless inspection scheme lacks a stable and reliable path planner, and the "one room, one vehicle" mode has high costs. This study aims to propose a new global path planner to address the aforementioned issues.

# 2.Methods
## 2.1 Environmental analysis and mapping
  Represent the poultry house environment as a global occupancy grid map, and build a semi-structured topological network based on it to provide a foundation for path planning.
## 2.2 Problem description
  The path planning of poultry house inspection vehicles needs to take into account requirements such as obstacle avoidance, maintaining straight driving, and smooth paths. The traditional HA algorithm has problems such as slow planning speed and high resource consumption. This study proposes the TSO-HA algorithm and TSO-HA * - Net hybrid path planner to meet the requirements.
## 2.3 Overall framework
  TSO-HA*-Net consists of the underlying TSO-HA algorithm and the upper layer topology planning. Firstly, determine the topology nodes and connections, use TSO-HA algorithm for path pre planning, construct a semi-structured topology network and cost matrix, and then use Dijkstra algorithm to plan the complete inspection route.
## 2.4 TSO-HA Bottom level path planner
  Improved HA * algorithm, using specific node expansion strategy, distance reference tree (DRT), and simplified occupancy grid template to enhance computational efficiency, used for building semi-structured topology networks.
## 2.5 Upper level topology planning
  Introduce connection points to enable vehicles to smoothly enter and exit the network, manage path points using Hash Maps, construct a cost matrix based on TSO-HA * path length, and use Dijkstra's algorithm to search for the shortest path in the network to determine the inspection route.

# 3.Experiment and data analysis
  Conduct comparative experiments based on the Jetson Orin NX platform to verify algorithm performance.
## 3.1 H2 Calculation
  The TSO-HA algorithm using DRT performs excellently in terms of planning success rate, computation time, and resource consumption, with significant advantages over LDP-MAP and A-dis.
## 3.2 Line collision detection
  The improved occupancy grid template is more efficient than the OBB collision detection algorithm, reducing front-end planning time, and pre collision detection can avoid planning failures and improve planning efficiency.
## 3.3 Inspection route planning based on TSO-HA Net
  The inspection route planning based on TSO-HA Net has excellent performance, which can achieve in house and inter house inspections, reduce vehicle demand, and has short calculation time. It can also support multi vehicle collaborative inspections.

# 4.Conclusion
  The combination of TSO-HA * - Net and topology planning shows high efficiency in poultry house experiments. The TSO-HA algorithm significantly reduces computation time and resource consumption, and TSO-HA * - Net only takes 546.62ms to plan the global route, which can meet the needs of poultry house inspection and replace traditional navigation methods. In the future, it will integrate positioning and navigation capabilities to achieve more accurate execution of inspection tasks.



# 💫Contact

  If you have any questions, please feel free to contact: **Yueping Sun** ``sunypujs@ujs.edu.cn``



<img src="" width="100%" height="100%"> 

<p align="center"></p>
