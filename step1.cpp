#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <list>
#include <string>
#include <cstring>
#include <sstream>
#include <typeinfo>


using namespace std;

struct point{
    int nodeId;
    double x;
    double y;
    point() : nodeId(0), x(0), y(0) {};
};
class Graph{

    //int N;
    vector<vector<int> > edges;
    vector<point> nodes;

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
        for(size_t i = 0; i < nodes.size(); i++) {
            cout << i << ": ";
            for (size_t j=0; j < edges[i].size();j++) {
                int index = edges[i][j];
                cout << nodes[index].x << "," << nodes[index].y << " ";
            }
            cout << endl;
        }
    }

    void printNodes(){
        for(int i = 0; i < nodes.size(); i++) {
            cout << "(" << nodes[i].x << "," << nodes[i].y << ") ";
        }
        cout << endl;
    }
};

// Declare Graph g
Graph g;

void tokenize(string const &str, const char* delim,
              vector<string> &out)
{
    char *token = strtok(const_cast<char*>(str.c_str()), delim);
    while (token != NULL)
    {
        out.push_back(string(token));
        token = strtok(NULL, delim);
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
    //string filename = "grid_map.txt";
    ifstream f(argv[1]);
    string line;
    unsigned int lineNo = 0;
    if(!f.is_open()) {
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
        //cout << setw(6 ) << lineNo << ": " << line << endl;
    }
    g.printNodes();
    g.printAdjList();
    //Graph g(4);
    //g.addEdge(0,1);
    //g.addEdge(0, 2);
    //g.addEdge(2,3);
    //g.addEdge(1,2);
    //g.printAdjList();

    f.close();
    return 0;
}

