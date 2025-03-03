#ifndef Dijkstra_h
#define Dijkstra_h
#include "Graph.h"

// Dijkstra算法
template <class T, class E>
void Dijkstra(Graphlnk<T, E> &G, int v, E dist[], int path[]) {
    // Graph是一个带权有向图，dist[]是当前求到的从顶点v到顶点j的最短路径长度，同时用数组
    // path[]存放求到的最短路径
    int n = G.numberOfVertices(); // 顶点数
    bool *s = new bool[n]; // 最短路径顶点集
    int i, j, k, u;
    E w, min;

    for(i = 0; i < n; i++) {
        dist[i] = G.getWeight(v,i); // 数组初始化，获取(v,i)边的权值
        s[i] = false; // 该顶点未被访问过
        if(i != v && dist[i] < G.maxValue) // 顶点i是v的邻接顶点
            path[i] = v; // 将v标记为顶点i的最短路径
        else
            path[i] = -1; // 说明该顶点i与顶点v没有边相连
    }
    s[v] = true; // 标记为访问过，顶点v加入s集合中
    dist[v] = 0;
    for(i = 0; i < n-1; i++) {
        min = G.maxValue;
        u = v; // 选不在生成树集合s[]中的顶点
        // 1.找v的权值最小且未被访问过的邻接顶点w,<v,w>
        for(j = 0; j < n; j++) {
            if(s[j] == false && dist[j] < min) {
                u = j;
                min = dist[j];
            }
        }
        s[u] = true; // 将顶点u加入到集合s
        for(k = 0; k < n; k++) { // 修改
            w = G.getWeight(u, k);
            if(s[k] == false && w < G.maxValue && dist[u] + w < dist[k]) {
                // 顶点k未被访问过，且从v->u->k的路径比v->k的路径短
                dist[k] = dist[u] + w;
                path[k] = u; // 修改到k的最短路径
            }
        }
    }
}

// 从path数组读取最短路径的算法
template <class T, class E>
void printShortestPath(Graphlnk<T, E> &G, int v, E dist[], int path[]) {
    int i, j, k, n = G.numberOfVertices();
    int *d = new int[n];

    cout << "从顶点" << G.getValue(v) << "到其他各顶点的最短路径为：" << endl;
    //cout  << "54" << endl;
    for(i = 0; i < n; i++) {
        //cout  << "54.1" << endl;
        if(i != v) { // 如果不是顶点v
            j = i;
            k = 0;
            //cout  << "54.4=(i=" << i<<", j="<<j<<", k="<<k<<")" << endl;
            while(j != v) {
                //cout  << "54.5" << "( j=" << j << ", path[j]=" << path[j] << ")" << endl;
                if (0 > path[j]) {
                    break;
                }
                d[k++] = j;
                j = path[j];
            }
            //cout  << "63" << endl;
            cout  << "顶点" << G.getValue(i) << "的最短路径为：" << G.getValue(v);
            //cout  << "65" << endl;
            while(k > 0) {
                cout << "->" << G.getValue(d[--k]);
            //cout  << "69" << endl;
            }
            cout << "，最短路径长度为：" << dist[i] << endl;
        }
    }
}

// 从path数组读取最短路径的算法
template <class T, class E>
void printShortestPath_A_to_B(Graphlnk<T, E> &G, int v0, int v1, E dist[], int path[]) {
    int i, j, k, n = G.numberOfVertices();
    int *d = new int[n];

    cout << "从顶点" << G.getValue(v0) << "到其他各顶点的最短路径为：" << endl;
    //cout  << "54" << endl;
    for(i = 0; i < n; i++) {
        if (i != v1) {
            continue;
        }

        //cout  << "54.1" << endl;
        if(i != v0) { // 如果不是顶点v
            j = i;
            k = 0;
            //cout  << "54.4=(i=" << i<<", j="<<j<<", k="<<k<<")" << endl;
            while(j != v0) {
                //cout  << "54.5" << "( j=" << j << ", path[j]=" << path[j] << ")" << endl;
                if (0 > path[j]) {
                    break;
                }
                d[k++] = j;
                j = path[j];
            }
            //cout  << "63" << endl;
            cout  << "顶点" << G.getValue(i) << "的最短路径为：" << G.getValue(v0);
            //cout  << "65" << endl;
            while(k > 0) {
                cout << "->" << G.getValue(d[--k]);
            //cout  << "69" << endl;
            }
            cout << "，最短路径长度为：" << dist[i] << endl;
        }
    }
}

const int maxSize = 40;

// 求有向网G的 u0 顶点到其余顶点的最短路径
template <class T, class E>
void PrintPath_A_to_All(Graphlnk<T, E> &G, const T u0)
{
    int dist[maxSize], path[maxSize], v0;

    v0 = G.getVertexPos(u0); // 取得起始顶点的位置
    // 我把dist数组放到有向图头文件中，方便建立有向图时，同时初始化dist数组
    Dijkstra(G, v0, dist, path); // 调用Dijkstra函数
    printShortestPath(G, v0, dist, path); // 输出到各个顶点的最短路径
}

// 求有向网G的 u0 顶点到其余顶点的最短路径
template <class T, class E>
void PrintPath_A_to_B(Graphlnk<T, E> &G, const T u0, const T u1)
{
    int dist[maxSize], path[maxSize], v0, v1;

    v0 = G.getVertexPos(u0); // 取得起始顶点的位置
    v1 = G.getVertexPos(u1); // 取得起始顶点的位置
    // 我把dist数组放到有向图头文件中，方便建立有向图时，同时初始化dist数组
    Dijkstra(G, v0, dist, path); // 调用Dijkstra函数
    printShortestPath_A_to_B(G, v0, v1, dist, path); // 输出到各个顶点的最短路径
}

#endif /* Dijkstra_h */