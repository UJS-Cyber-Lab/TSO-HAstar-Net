#pragma once
#ifndef _PATH_ADJUST_H
#define _PATH_ADJUST_H

#include "Graph.h"
#include "Dijkstra.h"
#include <vector>
#include <numeric>


class PathAdjust
{
private:
    Graphlnk<int, float> *G;

public:
    explicit PathAdjust(int count)
    {
        std::vector<int> vec(count);
        std::iota(vec.begin(), vec.end(), 0);
        G = new Graphlnk<int, float>(vec.size());
        G->InitVetex(vec);
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
        ret.push_back(0);
        float *dist = new float[G->numberOfVertices()];
        int *path = new int[G->numberOfVertices()];
        Dijkstra(*G, 0, dist, path);
        {
            int i,j,k,n = G->numberOfVertices();
            int *d = new int[n];
            for (i= n -1; i<n; i++){
                if(i!=0){
                    j=i;
                    k=0;
                    while(j!=0){
                        
                        d[k++] = j;
                        j      = path[j];
                    }
                    while(k>0) {
                        int dis = G->getValue(d[--k]);
                        ret.push_back(dis);
                    }
                }
            }
            delete[] d;
        }

        delete[] dist;
        delete[] path;
        return ret;
    }

};

#endif  //PATH_ADJUST_H
