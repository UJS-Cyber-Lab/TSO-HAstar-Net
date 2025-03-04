// Graph.h
#include <stdlib.h>

#ifndef Graph_h
#define Graph_h
#include <iostream>
#include <vector>
using namespace std;

const int DefaultVertices = 30;

template <class T, class E>
struct Edge {
    int dest;
    E cost;
    Edge<T, E> *link;
};

template <class T, class E>
struct Vertex {
    T data;
    Edge<T, E> *adj;
};

template <class T, class E>
class Graphlnk {
public:
    const E maxValue = 100000;
    Graphlnk(int sz=DefaultVertices);
    ~Graphlnk();
    void inputGraph();
    int add_edge(T e1, T e2, E weight);
    void outputGraph();
    T getValue(int i);
    E getWeight(int v1, int v2);
    bool insertVertex(const T& vertex);
    bool InitVetex(std::vector<int> const& vertexes);
    bool insertEdge(int v1, int v2, E weight);
    bool removeVertex(int v);
    bool removeEdge(int v1, int v2);
    int getFirstNeighbor(int v);
    int getNextNeighbor(int v,int w);
    int getVertexPos(const T vertex);
    int numberOfVertices();
private:
    int maxVertices;
    int numEdges;
    int numVertices;
    Vertex<T, E> * nodeTable;
};

template <class T, class E>
Graphlnk<T, E>::Graphlnk(int sz) {
    maxVertices = sz;
    numVertices = 0;
    numEdges = 0;
    nodeTable = new Vertex<T, E>[maxVertices];
    if(nodeTable == NULL) {
        exit(1);
    }
    for(int i = 0; i < maxVertices; i++)
        nodeTable[i].adj = NULL;
}

template <class T, class E>
Graphlnk<T, E>::~Graphlnk() {
    for(int i = 0; i < numVertices; i++) {
        Edge<T, E> *p = nodeTable[i].adj;
        while(p != NULL) {
            nodeTable[i].adj = p->link;
            delete p;
            p = nodeTable[i].adj;
        }
    }
    delete []nodeTable;
}

template <class T, class E>
int Graphlnk<T, E>::add_edge(T e1, T e2, E weight)
{
    int j, k;
    j = getVertexPos(e1);
    k = getVertexPos(e2);
    if(j == -1 || k == -1) {
        return -1;
    } else {
        insertEdge(j, k, weight);
        return 0;
    }
}

template <class T, class E>
void Graphlnk<T, E>::inputGraph() {
    int n, m;
    int i, j, k;
    T e1, e2;
    E weight;

    n = 7; 
    m = 11;

    // cin >> e1;
    insertVertex( 'A');  
    insertVertex( 'B');   
    insertVertex( 'C');   
    insertVertex( 'D'); 
    insertVertex( 'E');  
    insertVertex( 'F');  
    insertVertex( 'G'); 

    add_edge('B', 'C', 1 ); 
    add_edge('C', 'D', 1 ); 
    add_edge('D', 'E', 1 ); 
    add_edge('E', 'F', 1 ); 
    add_edge('C', 'G', 1 ); 
    add_edge('D', 'F', 1 );

    add_edge('C', 'B', 1 ); 
    add_edge('D', 'C', 1 ); 
    add_edge('E', 'D', 1 ); 
    add_edge('F', 'E', 1 ); 
    add_edge('G', 'C', 1 ); 

}

template <class T, class E>
void Graphlnk<T, E>::outputGraph() {
    int n, m, i;
    T e1, e2;
    E weight;
    Edge<T, E> *p;

    n = numVertices;
    m = numEdges;
    for(i = 0; i < n; i++) {
        p = nodeTable[i].adj;
        while(p != NULL) {
            e1 = getValue(i);
            e2 = getValue(p->dest);
            weight = p->cost;
            cout << "<" << e1 << ", " << e2 << ", " << weight << ">" << endl;
            p = p->link;
        }
    }
}

template <class T, class E>
T Graphlnk<T, E>::getValue(int i) {
    if(i >= 0 && i < numVertices)
        return nodeTable[i].data;
    return (T)NULL;
}

template <class T, class E>
E Graphlnk<T, E>::getWeight(int v1, int v2) {
    if(v1 != -1 && v2 != -1) {
        Edge<T , E> *p = nodeTable[v1].adj;
        while(p != NULL && p->dest != v2) {
            p = p->link;
        }
        if(p != NULL)
            return p->cost;
    }
    return maxValue;
}

template <class T, class E>
bool Graphlnk<T, E>::insertVertex(const T& vertex) {
    if(numVertices == maxVertices)
        return false;
    nodeTable[numVertices].data = vertex;
    numVertices++;
    return true;
}

template <class T, class E>
bool Graphlnk<T, E>::InitVetex(std::vector<int> const& vertexes) {
    for(auto i = 0; i < vertexes.size(); i++) {
        insertVertex(vertexes[i]);
    }   
    return true;
}

template <class T, class E>
bool Graphlnk<T, E>::insertEdge(int v1, int v2, E weight) {
    if(v1 >= 0 && v1 < numVertices && v2 >= 0 && v2 < numVertices) {
        Edge<T, E> *p = nodeTable[v1].adj;
        while(p != NULL && p->dest != v2)
            p = p->link;
        if(p != NULL)
            return false;
        p = new Edge<T, E>;
        p->dest = v2;
        p->cost = weight;
        p->link = nodeTable[v1].adj;
        nodeTable[v1].adj = p;
        numEdges++;
        return true;
    }
    return false;
}

template <class T, class E>
bool Graphlnk<T, E>::removeVertex(int v) {
    if(numVertices == 1 || v < 0 || v > numVertices)
        return false;

    Edge<T, E> *p, *s;
    while(nodeTable[v].adj != NULL) {
        p = nodeTable[v].adj;
        nodeTable[v].adj = p->link;
        delete p;
        numEdges--;
    }
    for(int i = 0; i < numVertices; i++) {
        if(i != v) {
            s = NULL;
            p = nodeTable[i].adj;
            while(p != NULL && p->dest != v) {
                s = p;
                p = p->link;
            }
            if(p != NULL) {
                if(s == NULL) {
                    nodeTable[i].adj = p->link;
                } else {
                    s->link = p->link;
                }
                delete p;
                numEdges--;
            }
        }
    }
    numVertices--;
    nodeTable[v].data = nodeTable[numVertices].data;
    nodeTable[v].adj = nodeTable[numVertices].adj;
    for(int i = 0; i < numVertices; i++) {
        p = nodeTable[i].adj;
        while(p != NULL && p->dest != numVertices)
            p = p->link;
        if(p != NULL)
            p->dest = v;
    }
    return true;
}

template <class T, class E>
bool Graphlnk<T, E>::removeEdge(int v1, int v2) {
    if(v1 != -1 && v2 != -1) {
        Edge<T, E> * p = nodeTable[v1].adj, *q = NULL;
        while(p != NULL && p->dest != v2) {
            q = p;
            p = p->link;
        }
        if(p != NULL) {
            if(q == NULL)
                nodeTable[v1].adj = p->link;
            else
                q->link = p->link;
            delete p;
            return true;
        }
    }
    return false;
}

template <class T, class E>
int Graphlnk<T, E>::getFirstNeighbor(int v) {
    if(v != -1) {
        Edge<T, E> *p = nodeTable[v].adj;
        if(p != NULL)
            return p->dest;
    }
    return -1;
}

template <class T, class E>
int Graphlnk<T, E>::getNextNeighbor(int v,int w) {
    if(v != -1) {
        Edge<T, E> *p = nodeTable[v].adj;
        while(p != NULL && p->dest != w)
            p = p->link;
        if(p != NULL && p->link != NULL)
            return p->link->dest;
    }
    return -1;
}

template <class T, class E>
int Graphlnk<T, E>::getVertexPos(const T vertex) {
    for(int i = 0; i < numVertices; i++)
        if(nodeTable[i].data == vertex)
            return i;
    return -1;
}

template <class T, class E>
int Graphlnk<T, E>::numberOfVertices() {
    return numVertices;
}

#endif /* Graph_h */