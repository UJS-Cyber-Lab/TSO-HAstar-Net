#pragma once
#ifndef _PATH_ADJUST_H
#define _PATH_ADJUST_H

#include "Graph.h"
#include "Dijkstra.h"
#include <vector>
#include <numeric>


//图论搜索最短路径，用于将路径拉直
class PathAdjust
{
private:
    Graphlnk<int, float> *G;

public:
    explicit PathAdjust(int count)
    {
        std::vector<int> vec(count);
/*它用于将连续的数值赋给指定范围内的元素，起始值是给定的第一个参数，然后依次递增直到范围结束。这个函数可以方便地用于初始化数组或容器中的元素，简化了代码编写。*/
        std::iota(vec.begin(), vec.end(), 0);//就是给顶点赋上从0开始的索引
        G = new Graphlnk<int, float>(vec.size());
        G->InitVetex(vec); //添加边？还是添加顶点
    }

    ~PathAdjust()
    {
        delete G;
    }

    void AddEdge(int e1, int e2, float len)
    {
        G->add_edge(e1, e2, len);
    }

    std::vector<int> GetShortestPath()
    {
        std::vector<int> ret;
        ret.push_back(0);//起点索引
        // 存储距离的数组
        float *dist = new float[G->numberOfVertices()]; //G.numberOfVertices(); //顶点数
        // 存储路径索引的数组
        int *path = new int[G->numberOfVertices()];
        Dijkstra(*G, 0, dist, path); //调用Dijkstra函数，起始顶点0
        //printShortestPath(*G, 0, dist, path); //输出到各个顶点的最短路径
        {
            int i,j,k,n = G->numberOfVertices();
            // d存放索引
            int *d = new int[n];

            //cout << "from" << G->getValue(0) << "to other path" << endl;
            // n为顶点数
            for (i= n -1; i<n; i++){
                if(i!=0){  //如果不是起始顶点
                    j=i;
                    k=0;
                    while(j!=0){
                        
                        d[k++] = j;
                        j      = path[j];
                    }
                    // cout << "vetex" << G->getValue(i) << "shortest path to" << G->getValue(0);
                    while(k>0) {
                        // 传入顶点索引，输出的也是(因为我们设置将索引值作为data名称了)
                        // float dis = G->getValue(d[--k]);
                        int dis = G->getValue(d[--k]);
                        // cout << "->" <<dis;
                        ret.push_back(dis);
                    }

                    // cout << "len is" << dist[i] << endl;
                }
            }
            delete[] d;
        }

        delete[] dist;
        delete[] path;
        return ret;//返回路点索引
    }

};



#endif  //PATH_ADJUST_H