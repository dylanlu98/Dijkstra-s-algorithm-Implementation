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

using namespace std;

static double inf = numeric_limits<double>::infinity();


struct point{
    int nodeId;
    double x;
    double y;
    point() : nodeId(0), x(0), y(0) {};
};

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
    vector<point> nodes;
    vector<bool> vis;
    vector<double> dis;
    vector<int> prev;
    //vector<Entry> path;
    //double inf = 10e15;
public:
    void addNode(int NodeId, double x, double y){
        point newPoint;
        newPoint.nodeId = NodeId;
        newPoint.x = x;
        newPoint.y = y;
        nodes.push_back(newPoint);
        edges.resize(nodes.size());
    }
    void addEdge(int x, int y){
        edges[x].push_back(y);
        edges[y].push_back(x);
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
            cerr << "Incorrect input argument" << endl;
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

int main(int argc, char const *argv[]) {
    if (argc != 4){
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

    vector<int> path = g.findBestPath(s,e);
    double shortestDistance = g.findBestDistance(e);
    for(int i=0; i<path.size();i++){
        cout << path[i] << " ";
    }
    cout << ": " << shortestDistance << endl;
    f.close();
    return 0;
}

