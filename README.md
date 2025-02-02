# TSO-HA*-Net
### TSO-HA*-Net: A Hybrid Global Path Planner for the Inspection Vehi-cle Used in Caged Poultry Houses
*Sample ROS-C++ code will be made available upon finalization in the COMPAG journal.*

### 💫Contact

If you have any questions, please feel free to contact: **Yueping Sun** ``sunypujs@ujs.edu.cn``

## 1.Introduction
A hybrid global path planner for the inspection vehicles that combines improved HA* with topological planning, establishing an intermediate planning scheme between point-to-point and full-coverage path planning.

<img src="https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_Project/blob/main/picture/1.%E4%BB%8B%E7%BB%8D/%E5%A4%8D%E6%9D%82%E9%B8%A1%E8%88%8D%E7%8E%AF%E5%A2%83.png" width="100%" height="100%"> 

<p align="center">
Fig. 1. The site structure of a large-scale caged poultry farm.
</p>

## 2.Introduction

![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_Project/blob/main/picture/2.6%E7%A7%8D%E4%B8%8D%E5%90%8C%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92%E5%9B%BE/%E6%8A%98%E7%BA%BFAtar%E5%9C%BA%E6%99%AF1.png)|![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_Project/blob/main/picture/2.6%E7%A7%8D%E4%B8%8D%E5%90%8C%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92%E5%9B%BE/%E6%8A%98%E7%BA%BFAstar%E5%9C%BA%E6%99%AF2.png)|![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_Project/blob/main/picture/2.6%E7%A7%8D%E4%B8%8D%E5%90%8C%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92%E5%9B%BE/%E6%8A%98%E7%BA%BFAstar%E5%9C%BA%E6%99%AF3.png)
:-:|:-:|:-:
(a)|(b)|(c)
![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_Project/blob/main/picture/2.6%E7%A7%8D%E4%B8%8D%E5%90%8C%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92%E5%9B%BE/%E6%8A%98%E7%BA%BFAstar%E5%9C%BA%E6%99%AF4.png)|![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_Project/blob/main/picture/2.6%E7%A7%8D%E4%B8%8D%E5%90%8C%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92%E5%9B%BE/%E6%8A%98%E7%BA%BFAstar%E5%9C%BA%E6%99%AF5.png)|![](https://github.com/UJS-Cyber-Lab/TSO-HAstar-Net_Project/blob/main/picture/2.6%E7%A7%8D%E4%B8%8D%E5%90%8C%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92%E5%9B%BE/%E6%8A%98%E7%BA%BFAstar%E5%9C%BA%E6%99%AF6.png)
(d)|(e)|(f)
