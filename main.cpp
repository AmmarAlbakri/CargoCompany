#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <cmath>
#include <climits>

using namespace std;

class Bolge
{
public:
    int totalCargoValue;
    string id;
};

class Ilce
{
public:
    char id[5];
    char name[20];
    char bolge[5];
};

class Packet
{
public:
    char id[5];
    char deliveryAddress[5];
    int volume;
    int earlyDelivery;
    int breakable;
    int corruptionValue;
    int numberOfDaysWaited;
    int CargoValue;
    void print()
    {
        cout << "ID: " << id << ", deliveryAddress: " << deliveryAddress << ", volume:" << volume << endl
             << "earlyDelivery: " << earlyDelivery << ", breakable: " << breakable << ", corruptionValue:" << corruptionValue << endl
             << "numberOfDaysWaited: " << numberOfDaysWaited << ", CargoValue: " << CargoValue << endl;
    }
};

Ilce *ilceler;
int numberOfIlceler = 0;
Packet *packets;
int numberOfPackets = 0;

struct node
{
    int weight;
    int dist;
    char id[8];
    struct node *next;
    bool visited;
};

struct vnode
{
    struct node *head;
    bool visited;
    char id[8];
    int numberOfConnections = 0;
    char ilceId[5];
};

struct multipleData
{
    string ilce;
    string *pointNames;
    int numberOFPoints;
    int **matrix;
};

void addBigEdge(char *first, char *second, int data);

class Graph
{
public:
    vnode *vertex;
    int numberOfPoints;
    int **matrix;
    vnode **points;

    Graph() {}

    Graph(int numberOfPoints, vnode input_points[13][13], int **matrix)
    {
        this->numberOfPoints = numberOfPoints;
        this->matrix = matrix;
        vertex = new vnode[numberOfPoints];
        points = new vnode *[13];
        for (int i = 0; i < 13; i++)
        {
            points[i] = new vnode[13];
        }
        int k = 0;
        for (int i = 0; i < 13; i++)
        {
            for (int j = 0; j < 13; j++)
            {
                if (matrix[i][j] != 0 && matrix[i][j] != -1)
                {
                    vertex[k].head = NULL;
                    vertex[k].visited = false;
                    strcpy(vertex[k].id, input_points[i][j].id);
                    strcpy(points[i][j].id, input_points[i][j].id);
                    k++;
                }
            }
        }
    }

    Graph(multipleData s)
    {
        this->numberOfPoints = s.numberOFPoints;
        this->matrix = s.matrix;
        vertex = new vnode[numberOfPoints];
        for (int i = 0; i < numberOfPoints; i++)
        {
            strcpy(vertex[i].id, s.pointNames[i].c_str());
            strcpy(vertex[i].ilceId, s.ilce.c_str());
        }
    }

    Graph(Graph *graph, Graph *smallGraphs[13])
    {
        vnode *Tnodes = new vnode;
        int totalNumberOfPoints = 0;
        for (int i = 0; i < 13; i++)
        {
            for (int j = 0; j < smallGraphs[i]->numberOfPoints; j++)
            {
                if (string(smallGraphs[i]->vertex[j].id).find("T") != string::npos || string(smallGraphs[i]->vertex[j].id).find("M") != string::npos)
                {
                    Tnodes[totalNumberOfPoints] = smallGraphs[i]->vertex[j];
                    totalNumberOfPoints++;
                    Tnodes = (vnode *)realloc(Tnodes, (totalNumberOfPoints + 1) * sizeof(vnode));
                }
            }
        }
        for (int i = 0; i < graph->numberOfPoints; i++)
        {

            Tnodes[totalNumberOfPoints] = graph->vertex[i];
            totalNumberOfPoints++;
            Tnodes = (vnode *)realloc(Tnodes, (totalNumberOfPoints + 1) * sizeof(vnode));
        }

        this->vertex = new vnode[numberOfPoints];
        this->numberOfPoints = totalNumberOfPoints;
        this->vertex = Tnodes;
    }

    void addedge(char *id)
    {

        for (int i = 0; i < 13; i++)
        {
            for (int j = 0; j < 13; j++)
            {
                if (matrix[i][j] != 0)
                {

                    if ((string)id == points[i][j].id)
                    {
                        for (int m = 0; m < numberOfPoints; m++)
                        {
                            if (vertex[m].id == id)
                            {
                                node *newnode = new node();
                                newnode->weight = matrix[j][i];
                                strcpy(newnode->id, points[j][i].id);
                                newnode->next = vertex[m].head;
                                vertex[m].head = newnode;
                                vertex[m].numberOfConnections++;
                            }
                        }
                    }
                }
            }
        }
    }

    void addSmallEdge(char *id)
    {
        for (int i = 0; i < numberOfPoints; i++)
        {
            if (vertex[i].id == id)
            {
                for (int j = 0; j < numberOfPoints; j++)
                {
                    if (matrix[i][j] != 0 && matrix[i][j] != -1)
                    {
                        node *newnode = new node();
                        newnode->weight = matrix[i][j];
                        strcpy(newnode->id, vertex[j].id);
                        newnode->next = vertex[i].head;
                        vertex[i].head = newnode;
                        vertex[i].numberOfConnections++;
                        if (string(vertex[j].id).find("Y") != string::npos)
                        {
                            addBigEdge(vertex[j].id, vertex[i].id, newnode->weight);
                        }
                    }
                }
            }
        }
    }

    vnode findVertex(char id[8])
    {
        for (int i = 0; i < numberOfPoints; i++)
        {
            if (strcmp(vertex[i].id, id) == 0)
            {
                return vertex[i];
            }
        }
    }

    int findVertexInt(char id[8])
    {
        for (int i = 0; i < numberOfPoints; i++)
        {
            if (strcmp(vertex[i].id, id) == 0)
            {
                return i;
            }
        }
    }

    int findVertexInt(string id)
    {
        for (int i = 0; i < numberOfPoints; i++)
        {
            if (strcmp(vertex[i].id, id.c_str()) == 0)
            {
                return i;
            }
        }
    }

    void printgraph()
    {
        node *newnode;
        for (int i = 0; i < numberOfPoints; i++)
        {
            cout << "Adjacent of " << vertex[i].id;
            newnode = vertex[i].head;
            for (int j = 0; j < vertex[i].numberOfConnections; j++)
            {
                cout << " --- " << newnode->id << ":" << newnode->weight;
                newnode = newnode->next;
            }
            cout << endl;
        }
    }
};

Graph *graph;
Graph *smallGraphs[13];
Graph *bigGraph;
int **bigMatrix;

Packet *includedPackets = new Packet;
int numberOfItemsIncluded = 0;

Bolge *bolgeler = new Bolge[4];

void createGraph(Graph *graph)
{
    for (int i = 0; i < graph->numberOfPoints; i++) //creating edges
    {
        graph->addedge(graph->vertex[i].id);
    }
}

void createSmallGraph(Graph *graph)
{
    for (int i = 0; i < graph->numberOfPoints; i++) //creating edges
    {
        graph->addSmallEdge(graph->vertex[i].id);
    }
}

void createBigGraph(Graph *graph, Graph *smallGraphs[13])
{
    bigGraph = new Graph(graph, smallGraphs);
}

void addBigEdge(char *first, char *second, int data)
{
    for (int i = 0; i < graph->numberOfPoints; i++)
    {
        if (strcmp(graph->vertex[i].id, first) == 0)
        {
            node *newnode = new node();
            newnode->weight = data;
            strcpy(newnode->id, second);
            newnode->next = graph->vertex[i].head;
            graph->vertex[i].head = newnode;
            graph->vertex[i].numberOfConnections++;
        }
    }
}

void readFiles();
void calculateCargoValue(int);
int max(int a, int b) { return (a > b) ? a : b; }
void GetPacketsIntoTheCar();
void sortPacketsByReigon();
int **makeMatrix();
int isConnected(vnode v, vnode n);
struct PathData findShortestPathBetween2(char *, char *);
void allShortstPaths();
void calculateRoad();
void pathBetween2Ilce();
void pathBetween2Points();
void menu();

struct PathData
{
    int *dist = new int[bigGraph->numberOfPoints];
    int *parent = new int[bigGraph->numberOfPoints];
    int src, destination;
};

// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int minDistance(int dist[],
                bool sptSet[])
{

    // Initialize min value
    int min = INT_MAX, min_index;

    for (int v = 0; v < bigGraph->numberOfPoints; v++)
        if (sptSet[v] == false &&
            dist[v] <= min)
            min = dist[v], min_index = v;

    return min_index;
}
void printPath(int parent[], int j)
{
    // Base Case : If j is source
    if (parent[j] == -1)
    {
        return;
    }

    printPath(parent, parent[j]);

    printf("%s-> ", bigGraph->vertex[parent[j]].id);
}

// A utility function to print the constructed distance array
int printSolution(int dist[], int parent[], int src, int destination)
{
    printf("\n%s -> %s \t\t %d\t\t ",
           bigGraph->vertex[src].id, bigGraph->vertex[destination].id, dist[destination]);
    printPath(parent, destination);
    cout << bigGraph->vertex[destination].id;
}

// Function that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
struct PathData dijkstra(int **graph, int src, int destination)
{

    // The output array. dist[i]
    // will hold the shortest
    // distance from src to i
    int *dist = new int[bigGraph->numberOfPoints];

    // sptSet[i] will true if vertex
    // i is included / in shortest
    // path tree or shortest distance
    // from src to i is finalized
    bool *sptSet = new bool[bigGraph->numberOfPoints];

    // Parent array to store
    // shortest path tree
    int *parent = new int[bigGraph->numberOfPoints];

    // Initialize all distances as
    // INFINITE and stpSet[] as false
    for (int i = 0; i < bigGraph->numberOfPoints; i++)
    {
        parent[src] = -1;
        dist[i] = INT_MAX;
        sptSet[i] = false;
    }

    // Distance of source vertex
    // from itself is always 0
    dist[src] = 0;

    // Find shortest path
    // for all vertices
    for (int count = 0; count < bigGraph->numberOfPoints - 1; count++)
    {
        // Pick the minimum distance
        // vertex from the set of
        // vertices not yet processed.
        // u is always equal to src
        // in first iteration.
        int u = minDistance(dist, sptSet);

        // Mark the picked vertex
        // as processed
        sptSet[u] = true;

        // Update dist value of the
        // adjacent vertices of the
        // picked vertex.
        for (int v = 0; v < bigGraph->numberOfPoints; v++)

            // Update dist[v] only if is
            // not in sptSet, there is
            // an edge from u to v, and
            // total weight of path from
            // src to v through u is smaller
            // than current value of
            // dist[v]
            if (!sptSet[v] && graph[u][v] &&
                dist[u] + graph[u][v] < dist[v])
            {
                parent[v] = u;
                dist[v] = dist[u] + graph[u][v];
            }
    }

    // print the constructed
    // distance array
    // printSolution(dist, bigGraph->numberOfPoints, parent, src, destination);

    struct PathData path;
    path.destination = destination;
    path.src = src;
    path.dist = dist;
    path.parent = parent;

    return path;
}

int main()
{
    readFiles();

    GetPacketsIntoTheCar();

    createGraph(graph);
    createBigGraph(graph, smallGraphs);
    bigMatrix = makeMatrix();

    allShortstPaths();

    sortPacketsByReigon();

    calculateRoad();

    menu();
}

void menu()
{
    int c;
    do
    {
        cout << endl;
        cout << "------------------------------------------" << endl
             << "1.Print shortest path between any two points (id, id)." << endl;
        cout << "2.Print shortest path between any two Ilce (id, id)." << endl;
        cout << "3.Print all graph connections." << endl;
        cout << "0. Exit" << endl;
        cin >> c;
        switch (c)
        {
        case 1:
            pathBetween2Points();
            break;
        case 2:
            pathBetween2Ilce();
            break;
        case 3:
            bigGraph->printgraph();
            break;
        default:
            break;
        }
    } while (c != 0);
}

void readIlceler()
{
    ifstream file;
    string line, value;
    int place, start = 0;
    ilceler = new Ilce();

    file.open("Ilceler.txt");
    while (file >> line)
    {
        // ID
        place = line.find(",", start);
        value = line.substr(start, place - start);
        start = place + 1;
        strcpy(ilceler[numberOfIlceler].id, value.c_str());

        // NAME
        place = line.find(",", start);
        value = line.substr(start, place - start);
        start = place + 1;
        strcpy(ilceler[numberOfIlceler].name, value.c_str());

        // BOLGE
        place = line.find(",", start);
        value = line.substr(start, place - start);
        start = place + 1;
        strcpy(ilceler[numberOfIlceler].bolge, value.c_str());

        start = place = 0;
        value = "";

        numberOfIlceler++;
        ilceler = (Ilce *)realloc(ilceler, (numberOfIlceler + 1) * sizeof(Ilce));
    }
    file.close();
}

void readPackets()
{
    ifstream file;
    string line, value;
    int place, start = 0;
    packets = new Packet();

    file.open("KargoPaketleri.txt");
    while (file >> line)
    {
        // ID
        place = line.find(",", start);
        value = line.substr(start, place - start);
        start = place + 1;
        strcpy(packets[numberOfPackets].id, value.c_str());

        // DELIVERY ADDRESS
        place = line.find(",", start);
        value = line.substr(start, place - start);
        start = place + 1;
        strcpy(packets[numberOfPackets].deliveryAddress, value.c_str());

        // VOLUME
        place = line.find(",", start);
        value = line.substr(start, place - start);
        start = place + 1;
        packets[numberOfPackets].volume = stoi(value);

        // EARLY DELIVERY
        place = line.find(",", start);
        value = line.substr(start, place - start);
        start = place + 1;
        packets[numberOfPackets].earlyDelivery = stoi(value);

        // BREAKABLE
        place = line.find(",", start);
        value = line.substr(start, place - start);
        start = place + 1;
        packets[numberOfPackets].breakable = stoi(value);

        // CORRUPTION VALUE
        place = line.find(",", start);
        value = line.substr(start, place - start);
        start = place + 1;
        packets[numberOfPackets].corruptionValue = stoi(value);

        // NUMBER OF DAYS WAITED
        place = line.find(",", start);
        value = line.substr(start, place - start);
        start = place + 1;
        packets[numberOfPackets].numberOfDaysWaited = stoi(value);

        // CARGO VALUE
        calculateCargoValue(numberOfPackets);

        start = place = 0;
        value = "";

        numberOfPackets++;
        packets = (Packet *)realloc(packets, (numberOfPackets + 1) * sizeof(Packet));
    }
    file.close();
}

int **readUrfaRoadInfo()
{
    ifstream file;
    string line, value;
    int place, start = 0;

    int **matrix = new int *[13];
    for (int i = 0; i < 13; i++)
    {
        matrix[i] = new int[13];
    }
    int row = 0, col;

    file.open("UrfaYolBilgisi.txt");
    while (file >> line)
    {
        for (col = 0; col < 13; col++)
        {
            place = line.find(",", start);
            value = line.substr(start, place - start);
            start = place + 1;
            if (value.compare("-") == 0)
            {
                matrix[row][col] = -1;
                continue;
            }
            else
            {
                matrix[row][col] = stoi(value);
            }
        }
        start = place = 0;
        value = "";
        row++;
    }

    return matrix;
}

string **readUrfaRoadInfoID()
{
    ifstream file;
    string line, value;
    int place, start = 0;

    string **matrix = new string *[13];
    for (int i = 0; i < 13; i++)
    {
        matrix[i] = new string[13];
    }
    int row = 0, col;

    file.open("UrfaYolBilgisiID.txt");
    while (file >> line)
    {
        for (col = 0; col < 13; col++)
        {
            place = line.find(",", start);
            value = line.substr(start, place - start);
            start = place + 1;
            if (value.compare("-") == 0)
            {
                matrix[row][col] = "-";
                continue;
            }
            else
            {
                matrix[row][col] = value;
            }
        }
        start = place = 0;
        value = "";
        row++;
    }

    return matrix;
}

void readDeliveryAddresses()
{
    ifstream file;
    string line, value;
    int place, start = 0;
    file.open("TeslimatAdresiUzaklikBilgileri.txt");

    string ilce;
    string pointNames[40];
    int pointNumber;

    struct multipleData objects[13];
    int numberOfObjects = 0;

    while (file >> line)
    {
        pointNumber = 0;
        ilce = line;
        file >> line;
        while (line.find(",", start) != string::npos)
        {
            place = line.find(",", start);
            value = line.substr(start, place - start);
            start = place + 1;
            if (value.find("Y") != string::npos)
            {
                value = ilce + "-" + value;
            }
            pointNames[pointNumber] = value;
            pointNumber++;
        }
        place = line.find(",", start);
        value = line.substr(start, place - start);
        start = place + 1;
        if (value.find("Y") != string::npos)
        {
            value = ilce + "-" + value;
        }
        pointNames[pointNumber] = value;
        pointNumber++;

        int matrix[13][13];
        for (int i = 0; i < pointNumber; i++)
        {
            start = place = 0;
            value = "";
            file >> line;
            for (int j = 0; j < pointNumber; j++)
            {
                place = line.find(",", start);
                value = line.substr(start, place - start);
                start = place + 1;

                if (value.compare("-") == 0)
                {
                    matrix[i][j] = -1;
                    continue;
                }
                else
                {
                    matrix[i][j] = stoi(value);
                }
            }
        }

        objects[numberOfObjects].ilce = ilce;
        objects[numberOfObjects].matrix = new int *[pointNumber];
        for (int k = 0; k < pointNumber; k++)
        {
            objects[numberOfObjects].matrix[k] = new int[pointNumber];
        }
        for (int i = 0; i < pointNumber; i++)
        {
            for (int j = 0; j < pointNumber; j++)
            {
                objects[numberOfObjects].matrix[i][j] = matrix[i][j];
            }
        }
        objects[numberOfObjects].pointNames = new string[pointNumber];
        objects[numberOfObjects].numberOFPoints = pointNumber;
        for (int i = 0; i < pointNumber; i++)
        {
            objects[numberOfObjects].pointNames[i] = pointNames[i];
        }
        numberOfObjects++;
    }

    for (int n = 0; n < numberOfObjects; n++)
    {
        smallGraphs[n] = new Graph(objects[n]);
        createSmallGraph(smallGraphs[n]);
    }
}

void readFiles()
{
    readIlceler();
    readPackets();
    int **matrix = readUrfaRoadInfo();
    string **matrixID = readUrfaRoadInfoID();
    int numberOfPoints = 0;
    vnode points[13][13];

    for (int i = 0; i < 13; i++)
    {
        int numberOfPointsInIlce = 0;
        for (int j = 0; j < 13; j++)
        {
            if (matrix[i][j] == 0 || matrix[i][j] == -1)
            {
                continue;
            }
            else
            {
                strcpy(points[i][j].id, matrixID[i][j].c_str());
                numberOfPoints++;
            }
        }
    }

    graph = new Graph(numberOfPoints, points, matrix);
    readDeliveryAddresses();
}

void calculateCargoValue(int index)
{
    packets[index].CargoValue = packets[index].earlyDelivery ? 50 : 0 + packets[index].breakable ? 30 : 0 + packets[index].corruptionValue + (pow(packets[index].numberOfDaysWaited, 2) * 5);
}

int *printknapSack(int W, int wt[], int val[], int n)
{
    int i, w;
    int **K = new int *[n + 1];
    for (int i = 0; i < n + 1; i++)
    {
        K[i] = new int[W + 1];
    }

    // Build table K[][] in bottom up manner
    for (i = 0; i <= n; i++)
    {
        for (w = 0; w <= W; w++)
        {
            if (i == 0 || w == 0)
            {
                K[i][w] = 0;
            }
            else if (wt[i - 1] <= w)
            {
                K[i][w] = max(val[i - 1] +
                                  K[i - 1][w - wt[i - 1]],
                              K[i - 1][w]);
            }
            else
            {
                K[i][w] = K[i - 1][w];
            }
        }
    }

    // stores the result of Knapsack
    int res = K[n][W];
    cout << "Total amount of cargo Value = " << res << endl;

    // store the value of the items included
    int *includedWeights = new int;

    w = W;
    for (i = n; i > 0 && res > 0; i--)
    {

        // either the result comes from the top
        // (K[i-1][w]) or from (val[i-1] + K[i-1]
        // [w-wt[i-1]]) as in Knapsack table. If
        // it comes from the latter one/ it means
        // the item is included.
        if (res == K[i - 1][w])
            continue;
        else
        {
            // This item is included.
            includedWeights[numberOfItemsIncluded] = wt[i - 1];
            (numberOfItemsIncluded)++;
            includedWeights = (int *)realloc(includedWeights, (numberOfItemsIncluded + 1) * sizeof(int));
            // Since this weight is included its
            // value is deducted
            res = res - val[i - 1];
            w = w - wt[i - 1];
        }
    }
    return includedWeights;
}

void GetPacketsIntoTheCar()
{
    int maxWeight = 150; // of the car 150m3

    int *weights = new int[numberOfPackets];
    int *values = new int[numberOfPackets];

    for (int i = 0; i < numberOfPackets; i++)
    {
        weights[i] = packets[i].volume;
        values[i] = packets[i].CargoValue;
    }
    int *includedWeights = new int;
    includedWeights = printknapSack(maxWeight, weights, values, numberOfPackets);

    int k = 0;

    cout << "The included Packets are:" << endl;
    cout << "\tID\t| TeslimatAdresi\t| Hacim(m3)\t| Kargo degeri\t| Endeks" << endl;
    cout << "---------------------------------------------------------------------------------" << endl;

    for (int i = 0; i < numberOfItemsIncluded; i++)
    {
        for (int j = 0; j < numberOfPackets; j++)
        {
            if (packets[j].volume == includedWeights[i])
            {
                includedPackets[k] = packets[j];
                cout << k + 1 << "\t" << includedPackets[k].id << "\t| " << includedPackets[k].deliveryAddress << "\t\t\t| " << includedPackets[k].volume << "\t\t| "
                     << includedPackets[k].CargoValue << "\t\t| " << includedPackets[k].CargoValue / includedPackets[k].volume << endl;
                includedWeights[i] = -1;
                packets[j].volume = -2;
                k++;
                includedPackets = (Packet *)realloc(includedPackets, sizeof(Packet) * (k + 1));
            }
        }
    }
    cout << "---------------------------------------------------------------------------------" << endl;
}

string getReigonByIlce(string ilce)
{
    for (int i = 0; i < 13; i++)
    {
        if (ilceler[i].id == ilce)
        {
            return ilceler[i].bolge;
        }
    }
}

struct TPoint
{
    string id;
    string ilce;
    string bolge;
};

struct PandT
{
    TPoint Tpoint;
    Packet packet;
};

void sortPacketsByReigon()
{

    bolgeler[0].id = "GD";
    bolgeler[1].id = "GB";
    bolgeler[2].id = "KD";
    bolgeler[3].id = "KB";

    bolgeler[0].totalCargoValue = bolgeler[1].totalCargoValue = bolgeler[2].totalCargoValue = bolgeler[3].totalCargoValue = 0;

    ifstream file;
    string line, value;
    int place, start = 0;
    file.open("TeslimatAdresiUzaklikBilgileri.txt");

    string ilce;

    struct TPoint Tpoints[40];
    int k = 0;
    int numberOfPoints;
    while (file >> line)
    {
        numberOfPoints = 0;
        ilce = line;
        file >> line;
        while (line.find(",", start) != string::npos)
        {
            place = line.find(",", start);
            value = line.substr(start, place - start);
            start = place + 1;
            numberOfPoints++;
            if (value.find("Y") == string::npos)
            {
                Tpoints[k].id = value;
                Tpoints[k].ilce = ilce;
                k++;
            }
        }
        numberOfPoints++;
        place = line.find(",", start);
        value = line.substr(start, place - start);
        start = place + 1;
        if (value.find("Y") == string::npos)
        {
            Tpoints[k].id = value;
            Tpoints[k].ilce = ilce;
            k++;
        }
        for (int i = 0; i < numberOfPoints; i++)
        {
            file >> line;
        }
    }

    for (int i = 0; i < k; i++)
    {
        Tpoints[i].bolge = getReigonByIlce(Tpoints[i].ilce);
    }

    struct PandT *pt = new PandT[numberOfItemsIncluded];

    for (int i = 0; i < numberOfItemsIncluded; i++)
    {
        for (int j = 0; j < k; j++)
        {
            if (strcmp(Tpoints[j].id.c_str(), includedPackets[i].deliveryAddress) == 0)
            {
                pt[i].Tpoint = Tpoints[j];
                pt[i].packet = includedPackets[i];
                if (pt[i].Tpoint.bolge == "GD")
                {
                    bolgeler[0].totalCargoValue += pt[i].packet.CargoValue;
                }
                else if (pt[i].Tpoint.bolge == "KD")
                {
                    bolgeler[2].totalCargoValue += pt[i].packet.CargoValue;
                }
                else if (pt[i].Tpoint.bolge == "GB")
                {
                    bolgeler[1].totalCargoValue += pt[i].packet.CargoValue;
                }
                else if (pt[i].Tpoint.bolge == "KB")
                {
                    bolgeler[3].totalCargoValue += pt[i].packet.CargoValue;
                }
            }
        }
    }

    for (int i = 0; i < 4; i++) // sort Areas bu cargo value
    {
        for (int j = i + 1; j < 4; j++)
        {
            if (bolgeler[i].totalCargoValue < bolgeler[j].totalCargoValue)
            {
                Bolge temp = bolgeler[i];
                bolgeler[i] = bolgeler[j];
                bolgeler[j] = temp;
            }
        }
    }

    cout << "Areas by order of cargo value:" << endl;
    for (int i = 0; i < 4; i++)
    {

        cout << bolgeler[i].id << " = " << bolgeler[i].totalCargoValue << endl;
    }
}

int **makeMatrix()
{

    int **bigMatrix = new int *[bigGraph->numberOfPoints];

    for (int i = 0; i < bigGraph->numberOfPoints; i++)
    {
        bigMatrix[i] = new int[bigGraph->numberOfPoints];
    }

    for (int i = 0; i < bigGraph->numberOfPoints; i++)
    {
        for (int j = 0; j < bigGraph->numberOfPoints; j++)
        {
            int w;
            if (w = isConnected(bigGraph->vertex[i], bigGraph->vertex[j]))
            {
                bigMatrix[i][j] = w;
            }
            else
            {
                bigMatrix[i][j] = 0;
            }
        }
    }
    return bigMatrix;
}

int isConnected(vnode v, vnode n)
{
    node *temp = v.head;
    for (int i = 0; i < v.numberOfConnections; i++)
    {
        if (strcmp(temp->id, n.id) == 0)
        {
            return temp->weight;
        }
        temp = temp->next;
    }
    return 0;
}

string *getIOPoints(string ilceID)
{
    string *IOPoints = new string[10];
    int n = 0;
    for (int i = 0; i < bigGraph->numberOfPoints; i++)
    {
        if (string(bigGraph->vertex[i].id).find(ilceID) != string::npos)
        {
            IOPoints[n] = bigGraph->vertex[i].id;
            n++;
        }
    }
    return IOPoints;
}

struct PathData **allShortestPaths = new struct PathData *[13];

void allShortstPaths()
{
    for (int i = 0; i < 13; i++)
    {
        allShortestPaths[i] = new struct PathData[13];
    }

    for (int i = 0; i < 13; i++)
    {
        for (int j = 0; j < 13; j++)
        {
            if (i == j)
            {
            }
            else
            {
                allShortestPaths[i][j] = findShortestPathBetween2(ilceler[i].id, ilceler[j].id);
            }
        }
    }
    cout << "All Shortest Paths between all Ilceler:" << endl;
    cout << "\t";
    for (int i = 0; i < 13; i++)
    {
        cout << ilceler[i].id << "\t";
    }
    cout << endl;
    for (int i = 0; i < 13; i++)
    {
        cout << ilceler[i].id << "\t";
        for (int j = 0; j < 13; j++)
        {
            if (i == j)
            {
                cout << 0 << "\t";
            }
            else
            {
                cout << allShortestPaths[i][j].dist[allShortestPaths[i][j].destination] << "\t";
            }
        }
        cout << endl;
    }
}

int getArraySize(string *arr)
{
    int i = 0;
    while (!arr[i].empty())
        ++i;
    return i;
}

struct PathData findShortestPathBetween2(char *id1, char *id2)
{
    string *startPoints;
    string *endPoints;
    startPoints = getIOPoints(id1);
    endPoints = getIOPoints(id2);
    int index1 = getArraySize(startPoints);
    int index2 = getArraySize(endPoints);
    struct PathData path;
    int minDestince = INT_MAX;
    struct PathData shortestPath;

    for (int j = 0; j < index1; j++)
    {
        for (int n = 0; n < index2; n++)
        {
            path = dijkstra(bigMatrix, bigGraph->findVertexInt(startPoints[j]), bigGraph->findVertexInt(endPoints[n]));
            if (path.dist[path.destination] <= minDestince)
            {
                minDestince = path.dist[path.destination];
                shortestPath = path;
            }
        }
    }
    return shortestPath;
}

void calculateRoad()
{
    int bolgeIndex = 0;
    int ilceIndex;
    int startIlce = 6;
    int shortestDestince = INT_MAX;
    struct PathData shortestPath;
    struct PathData path;
    cout << "shortest path to ";
    cout << "(" << bolgeler[bolgeIndex].id << ")";
    for (int i = 0; i < 13; i++)
    {
        if (ilceler[i].bolge == bolgeler[bolgeIndex].id)
        {
            if (allShortestPaths[startIlce][i].dist[allShortestPaths[startIlce][i].destination] < shortestDestince)
            {
                ilceIndex = i;
                shortestDestince = allShortestPaths[startIlce][i].dist[allShortestPaths[startIlce][i].destination];
                shortestPath = allShortestPaths[startIlce][i];
            }
        }
    }

    cout << " is " << ilceler[startIlce].id << " ---> " << ilceler[ilceIndex].id;
    path = dijkstra(bigMatrix, bigGraph->findVertexInt("M"), shortestPath.destination);
    printSolution(path.dist, path.parent, path.src, path.destination);
}

void pathBetween2Points()
{
    cout << "First Point id: (example:U03-Y2)" << endl;
    string id1;
    cin >> id1;
    cout << "Second Point id: (example:TA02)" << endl;
    string id2;
    cin >> id2;
    struct PathData path;
    path = dijkstra(bigMatrix, bigGraph->findVertexInt(id1), bigGraph->findVertexInt(id2));
    printSolution(path.dist, path.parent, path.src, path.destination);
}

void pathBetween2Ilce()
{
    cout << "First Ilce id: (example:U03)" << endl;
    string id1;
    cin >> id1;
    cout << "Second Ilce id: (example:U01)" << endl;
    string id2;
    cin >> id2;
    int index1, index2;
    struct PathData path;
    for (int i = 0; i < 13; i++)
    {
        if (ilceler[i].id == id1)
        {
            index1 = i;
        }
        if (ilceler[i].id == id2)
        {
            index2 = i;
        }
    }

    path = allShortestPaths[index1][index2];
    printSolution(path.dist, path.parent, path.src, path.destination);
}