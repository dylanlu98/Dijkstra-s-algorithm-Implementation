#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <list>
#include <string>
#include <cstring>
#include <sstream>
#include <queue>
#include <typeinfo>
#include <cmath>
//#include <bits/stdc++.h>

using namespace std;

static double inf = numeric_limits<double>::infinity();

struct point{
    int nodeId;
    double x;
    double y;
    point() : nodeId(0), x(0), y(0) {};
};

struct edge{
    point p1;
    point q1;
    //edge();
    //edge() : p1(p1), q1(q1) {};

};

// Given three collinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(point p, point q, point r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;

    return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(point p, point q, point r)
{
    // Source: https://www.geeksforgeeks.org/orientation-3-ordered-points/

    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

    if (abs(val - 0) < 0.001 ) return 0;  // collinear

    return (val > 0)? 1: 2; // clock or counterclock wise
}

// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect(point p1, point q1, point p2, point q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}

class Entry {
public:
    int nodeId;
    double cost;

    Entry(int nodeId, double cost) :
            nodeId(nodeId),
            cost(cost) {}

    bool operator<(const Entry &rhs) const {
        // we return true if the node's cost is greater than the oder
        if (cost != rhs.cost) {
            return cost > rhs.cost;
        } else {
            return nodeId > rhs.nodeId;
        }
    }
};

using namespace std;
class Path{
public:
    vector<int> nodeIds;
    double totalCost;

    Path(vector<int> nodeIds, double totalCost) :
            nodeIds(nodeIds),
            totalCost(totalCost) {}
};

class Graph{
    vector<vector<int> > edges;
    vector<vector<int> > badEdges;
    vector<point> nodes;
    vector<int> obstacles;
    vector<bool> vis;
    vector<double> dis;
    vector<int> prev;
public:
    void addNode(int NodeId, double x, double y){
        point newPoint;
        newPoint.nodeId = NodeId;
        newPoint.x = x;
        newPoint.y = y;
        nodes.push_back(newPoint);
        edges.resize(nodes.size());
        badEdges.resize(nodes.size());

    }
    void addEdge(int x, int y){
        edges[x].push_back(y);
        edges[y].push_back(x);
    }

    void addBadEdges(int x, int y){
        badEdges[x].push_back(y);
        badEdges[y].push_back(x);
    }

    void addObstacle(int x){
        obstacles.push_back(x);
    }
    void printAdjList(){
        for(int i = 0; i < nodes.size(); i++) {
            cout << i << ": ";
            for (int j=0; j < edges[i].size();j++) {
                int index = edges[i][j];
                cout << nodes[index].x << "," << nodes[index].y << " ";
            }
            cout << endl;
        }
    }

    void printObstacleAdjList(){
        for(int i = 0; i < badEdges.size(); i++) {
            cout << i << ": ";
            for (int j=0; j < badEdges[i].size();j++) {
                int index = badEdges[i][j];
                cout << index  << " ";
            }
            cout << endl;
        }
    }

    // Need an adjacency list of weighted graph 'edges',
    // number of nodes 'n', start node 's' and end node 'e'.
    vector<double> dijkstra(int s, int e){
        for (int i=0; i<nodes.size();i++){
            prev.push_back(-1);
        }
        // initialize a visited boolean vector with false for every node
        for (int i=0; i<nodes.size();i++){
            vis.push_back(false);
        }
        // initialize a distance vector with infinity for each node
        for (int i=0; i<nodes.size();i++){
            dis.push_back(inf);
        }
        // initialize the distance from start node to start node be zero
        dis.at(s) = 0.0;
        // declare an empty priority queue
        priority_queue< Entry > pq;
        // Initialize an Entry object with field nodeId and cost
        Entry entry(s, 0.0);
        // Initialize a path object with a vector field storing the path and a cost field
        pq.push(entry);
        while(!pq.empty()){
            Entry entry = pq.top();
            //path.push_back(entry);
            pq.pop();
            //double minValue = entry.cost;
            int startIndex = entry.nodeId;
            vis[startIndex] = true;
            // Set up an early end to optimize the algorithm
            //double startDistance = dis[startIndex];
            //if (startDistance < minValue){
            //    continue;
            //};

            for(int edgesForOneNode = 0; edgesForOneNode < edges[startIndex].size(); edgesForOneNode++){
                int edgeToIndex = edges[startIndex][edgesForOneNode];
                if(vis[edgeToIndex]){
                    continue; //ignore edge that has been used;
                }
                // compute edge cost
                double edgeCost = sqrt(pow(nodes[edgeToIndex].y-nodes[startIndex].y ,2) + pow(nodes[edgeToIndex].x-nodes[startIndex].x ,2));
                double newDist = dis[startIndex] + edgeCost;
                // Update distance for the node
                if(abs(newDist) < abs(dis[edgeToIndex])){
                    prev[edgeToIndex] = startIndex;
                    dis[edgeToIndex] = newDist;
                    // Print out the current path
                    Entry entry (edgeToIndex, newDist);
                    pq.push(entry);
                }
            }
        }
        // print the results of dijkstra algorithm
//        for(int i =0; i < dis.size(); i++) {
//            cout << "Distance to reach Node" << i << " is " << dis.at(i);
//            cout << endl;
//        }
        return(dis);
    }

    void printPath(vector<int> path){
        double total = 0;
        vector<int> new_path = path;

        for(int j= 1; j<new_path.size();j++) {
            total = 0;
            for (int i = 0; i < j; i++) {
                cout << new_path[i] << " ";
                if(i == 0){
                    total = total + sqrt(pow(nodes[new_path[i]].y-0 ,2) + pow(nodes[new_path[i]].x-0 ,2));
                }
                else{
                    total = total + sqrt(pow(nodes[new_path[i]].y-nodes[new_path[i-1]].y ,2) + pow(nodes[new_path[i]].x-nodes[new_path[i-1]].x ,2));
                }
            }
            cout << ": " << total << endl;
        }
    }

    vector<int> findBestPath(int s, int e){
        dijkstra(s,e);
        vector<int> path;
        if (dis[e] == inf){
            cerr << "No path exists" << endl;
            exit(EXIT_FAILURE);
        }
        for (int at = e; at != -1; at = prev[at]){
            path.push_back(at);
        }
        reverse(path.begin(), path.end());
        printPath(path);
        return path;
    }

    void printNodes(){
        for(int i = 0; i < nodes.size(); i++) {
            cout << "(" << nodes[i].x << "," << nodes[i].y << ") ";
        }
        cout << endl;
    }

    double findBestDistance(int e){
        return(abs(dis[e]));
    }
    //  A member function of Graph that takes a vector of nodes representing an
    //    obstacle, which would
    //    o construct an edge for each pair of nodes,
    //    o iterate over all edges in the graph and for each edge, check if it
    //      intersects any of the obstacle's edges and remove the graph edge if
    //      so.

    void removeObstacleEdge(){
        edge ed;
        point p;
        point p1;
        point q1;
        point p2;
        point q2;
        vector<edge> obstaclesEdges;

        for(int i = 0; i < obstacles.size(); i++){
            if(i == obstacles.size()-1){
                addBadEdges(obstacles[i], obstacles[0]);
                p1.x = nodes[obstacles[i]].x;
                p1.y = nodes[obstacles[i]].y;
                q1.x = nodes[obstacles[0]].x;
                q1.y = nodes[obstacles[0]].y;
                ed.p1 = p1;
                ed.q1 = q1;
                delEdge(obstacles[i], obstacles[0]);
                obstaclesEdges.push_back(ed);

                //cout << "edges" << obstacles[i] << "," << obstacles[0] << endl;
            }else{
                addBadEdges(obstacles[i], obstacles[i+1]);
                p1.x = nodes[obstacles[i]].x;
                p1.y = nodes[obstacles[i]].y;
                q1.x = nodes[obstacles[i+1]].x;
                q1.y = nodes[obstacles[i+1]].y;
                ed.p1 = p1;
                ed.q1 = q1;
                obstaclesEdges.push_back(ed);
                delEdge(obstacles[i], obstacles[i+1]);
                //cout << "edges" << obstacles[i] << "," << obstacles[i+1] << endl;
            }
        }

        for(int i = 0; i < nodes.size(); i++) {
            p2.x = nodes[i].x;
            p2.y = nodes[i].y;
            for (int j=0; j < edges[i].size();j++) {
                int index = edges[i][j];
                q2.x = nodes[index].x;
                q2.y = nodes[index].y;
                for(int k = 0; k<obstaclesEdges.size(); k++){
                    p1.x = obstaclesEdges[k].p1.x;
                    p1.y = obstaclesEdges[k].p1.y;
                    q1.x = obstaclesEdges[k].q1.x;
                    q1.y = obstaclesEdges[k].q1.y;
                    if(doIntersect(p1, q1, p2,q2)){
                        delEdge(i,index);
                    }
                }
            }
        }

        //Testing if the obstaclesEdges are correctly obtained
//        for(int i = 0; i< obstaclesEdges.size(); i++ ){
//            cout << "x1->"<< obstaclesEdges[i].p1.x;
//            cout << ", y1-> " << obstaclesEdges[i].p1.y;
//            cout << " x2->"<< obstaclesEdges[i].q1.x;
//            cout << ", y2-> " << obstaclesEdges[i].q1.y;
//            cout << endl;
//        }
        // Loop through the obstacle edges i.e 29 94 42

    }


    void delEdge(int u, int v)
    {
        for (int i = 0; i < edges[u].size(); i++) {
            if (edges[u][i] == v) {
                edges[u].erase(edges[u].begin() + i);
                break;
            }
        }

        for (int i = 0; i < edges[v].size(); i++) {
            if (edges[v][i] == u) {
                edges[v].erase(edges[v].begin() + i);
                break;
            }
        }
    }

    vector<int> printObstacle(){
        for(int i = 0; i < obstacles.size(); i++){
            cout << obstacles[i] << " ";
        }
        cout << endl;
        return obstacles;
    }
};

// Declare Graph g
Graph g;

void tokenize(string const &str, const char* delim,
              vector<string> &out)
{
    char *token = strtok(const_cast<char*>(str.c_str()), delim);
    while (token != nullptr)
    {
        out.push_back(string(token));
        token = strtok(nullptr, delim);
    }
}

void parseLineNodes(string line, unsigned int lineNo){
    unsigned int nodeNum;
    double x_cor;
    double y_cor;
    const char* delim = " ";
    // use while loop to check the getline() function condition
    vector<string> out;
    tokenize(line, delim, out);
    nodeNum = stoul(out.at(0));
    x_cor = stod(out.at(1));
    y_cor = stod(out.at(2));
    g.addNode(nodeNum, x_cor, y_cor);
    //cout << out.at(0) << endl;
    //for (auto &s: out) {
    //    std::cout << s << std::endl;
    //}
}

void parseLineEdges(string line, unsigned int lineNo){
    int nodeX;
    int nodeY;
    const char* delim = " ";
    // use while loop to check the getline() function condition
    vector<string> out;
    tokenize(line, delim, out);
    nodeX = stod(out.at(0));
    nodeY = stod(out.at(1));
    g.addEdge(nodeX, nodeY);
    //cout << out.at(0) << endl;
    //for (auto &s: out) {
    //    std::cout << s << std::endl;
    //}
}

void parseLineObstacle(string line, unsigned int lineNo){
    int nodeId;
    const char* delim = " ";
    vector<string> out;
    tokenize(line, delim, out);
    for(int i=0; i < out.size(); i++){
        nodeId = stoi(out[i]);
        g.addObstacle(nodeId);
    }
}

int main(int argc, char const *argv[]) {
    if (argc != 5){
        cerr << "Incorrect input argument" << endl;
        return 1;
    }
    int s = stoi(argv[2]);
    int e = stoi(argv[3]);
    //string filename = "grid_map.txt";
    ifstream f(argv[1]);
    string line;
    unsigned int lineNo = 0;
    if(!f.is_open()) {
        cerr << "cannot open file" << endl;
        return 1;
    }
    //skip first line "$nodes"
    getline(f, line);
    while(getline(f,line)) {
        if(line[0] == '$'){
            break;
        }
        parseLineNodes(line, lineNo);
        lineNo ++;
    }
    while(getline(f,line)) {
        lineNo ++;
        parseLineEdges(line, lineNo);
    }
    ifstream f2(argv[4]);
    if(!f2.is_open()) {
        cerr << "cannot open file" << endl;
        return 1;
    }
    getline(f2, line);
    while(getline(f2,line)) {
        parseLineObstacle(line, lineNo);
        lineNo ++;
    }
    //g.removeObstacleEdge();
    g.removeObstacleEdge();
    //g.printAdjList();
    vector<int> path = g.findBestPath(s,e);
    double shortestDistance = g.findBestDistance(e);
    for(int i=0; i<path.size();i++){
        cout << path[i] << " ";
    }
    cout << ": " << shortestDistance << endl;
    //g.printObstacle();
    //g.printObstacleAdjList();

    f.close();
    return 0;
}

