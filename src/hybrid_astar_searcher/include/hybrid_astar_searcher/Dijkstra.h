#ifndef Dijkstra_h
#define Dijkstra_h
#include "Graph.h"


template <class T, class E>
void Dijkstra(Graphlnk<T, E> &G, int v, E dist[], int path[]) {
    int n = G.numberOfVertices();
    bool *s = new bool[n];
    int i, j, k, u;
    E w, min;

    for(i = 0; i < n; i++) {
        dist[i] = G.getWeight(v,i);
        s[i] = false;
        if(i != v && dist[i] < G.maxValue)
            path[i] = v;
        else
            path[i] = -1;
    }
    s[v] = true;
    dist[v] = 0;
    for(i = 0; i < n-1; i++) {
        min = G.maxValue;
        u = v;
        for(j = 0; j < n; j++) {
            if(s[j] == false && dist[j] < min) {
                u = j;
                min = dist[j];
            }
        }
        s[u] = true;
        for(k = 0; k < n; k++) {
            w = G.getWeight(u, k);
            if(s[k] == false && w < G.maxValue && dist[u] + w < dist[k]) {
                dist[k] = dist[u] + w;
                path[k] = u;
            }
        }
    }
}

template <class T, class E>
void printShortestPath(Graphlnk<T, E> &G, int v, E dist[], int path[]) {
    int i, j, k, n = G.numberOfVertices();
    int *d = new int[n];

    for(i = 0; i < n; i++) {
        if(i != v) {
            j = i;
            k = 0;
            while(j != v) {
                if (0 > path[j]) {
                    break;
                }
                d[k++] = j;
                j = path[j];
            }
            while(k > 0) {
                cout << "->" << G.getValue(d[--k]);
            }
        }
    }
}

template <class T, class E>
void printShortestPath_A_to_B(Graphlnk<T, E> &G, int v0, int v1, E dist[], int path[]) {
    int i, j, k, n = G.numberOfVertices();
    int *d = new int[n];
    for(i = 0; i < n; i++) {
        if (i != v1) {
            continue;
        }
        if(i != v0) {
            j = i;
            k = 0;
            while(j != v0) {
                if (0 > path[j]) {
                    break;
                }
                d[k++] = j;
                j = path[j];
            }
            while(k > 0) {
                cout << "->" << G.getValue(d[--k]);
            }
        }
    }
}

const int maxSize = 40;

template <class T, class E>
void PrintPath_A_to_All(Graphlnk<T, E> &G, const T u0)
{
    int dist[maxSize], path[maxSize], v0;
    v0 = G.getVertexPos(u0);
    Dijkstra(G, v0, dist, path);
    printShortestPath(G, v0, dist, path);
}

template <class T, class E>
void PrintPath_A_to_B(Graphlnk<T, E> &G, const T u0, const T u1)
{
    int dist[maxSize], path[maxSize], v0, v1;

    v0 = G.getVertexPos(u0);
    v1 = G.getVertexPos(u1); 
    Dijkstra(G, v0, dist, path); 
    printShortestPath_A_to_B(G, v0, v1, dist, path);
}

#endif /* Dijkstra_h */