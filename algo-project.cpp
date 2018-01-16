#include<iostream>
#include <ctime>
#include <cstdlib>
#include <utility>
#include <time.h>
#include <iomanip>

using namespace std;

//----------------------------Heap Begins-----------------------------------------------
struct Node
{
    int src, dest, weight;
};

void max_heapify (int H[ ],struct Node *node,int I[], int i, int N)//
{
    int left = 2*i;                   //left child
    int right = 2*i +1;           //right child
    int largest;
    if(left<= N and node[H[left]].weight > node[H[i]].weight ) //
          largest = left;
    else
         largest = i;
    if(right <= N and node[H[right]].weight > node[H[largest]].weight )//if(right <= N and D[H[right]] > D[H[largest]] )
        largest = right;
    if(largest != i )
    {
        swap (H[i] , H[largest]);
        I[H[i]] = i;
        I[H[largest]] = largest;
        
        max_heapify (H,node,I, largest,N);
    } 
 }
 
/*
void build_maxheap (int H[ ],int D[], int N)
{
    for(int i = N/2 ; i >= 1 ; i-- )
    {
        max_heapify (H,D, i, N) ;
    }
}
*/
int extract_maximum (int H[],struct Node *node, int I[], int *length)
{
    if(length == 0)
    {
        cout<< "Can’t remove element as queue is empty";
        return -1;
    }
    int max = H[1];
    //D[1] = -1;
    swap(H[1], H[*length]);
    I[H[1]] = 1;
    I[H[*length]] = *length;
    *length = *length -1;
    max_heapify(H,node,I, 1,*length);
    return max;
}

int searchVertex (int H[], int v, int length)
{
    int iter;
    for(iter = 1; iter<=length;iter++)
        if(H[iter] == v)
            return iter;
    return -1;
}

void increase_value(int H[],struct Node *node,int I[], int v, int bw, int length)
{
    int i = I[v];
    if(bw < node[H[i]].weight )
    {
        cout<<"New bw is less than current bw, can’t be inserted" <<endl;
        return;
    }
    node[v].weight = bw;
    while( i > 1 and (node[ H[i/2] ].weight) < (node[ H[i] ].weight))
    {
        swap(H[i], H[ i/2 ]);
        I[H[i]] = i;
        I[H[i/2]] = i/2;
        i = i/2;
    }
}

void insert_value (int H[],struct Node *node,int I[], int vertex, int bw, int* length)
{
    *length = *length + 1;
    H[*length] = vertex; 
    node[vertex].weight = -1;
    I[vertex] = *length;
    increase_value(H,node,I, vertex, bw,*length);
}

void sort(int H[],struct Node *node,int I[], int length)
{
    int i,size = length;
    for(i = length; i>=2; i--)
    {
        swap(H[i],H[1]);
        I[H[i]] = i;
        I[H[1]] = 1;
        size--;
        max_heapify(H,node,I,1,size);
    }
}
//---------------------------------------------Heap Ends--------------------------------------



// Adjacency list node
struct AdjListNode
{
    int dest;
    int weight;
    struct AdjListNode* next;
};
 
// Adjacency list
struct AdjList
{
    struct AdjListNode *head;  // pointer to head node of list
};
 
// Array of adjacency lists.
// No. of vertices -- named from 1 to V
struct Graph
{
    int V;
    struct AdjList* array;
    int* edgeCount;
    int E;
};

struct AdjListNode* newAdjListNode(int dest)
{
    struct AdjListNode* newNode = new AdjListNode;
    newNode->dest = dest;
    newNode->next = NULL;
    return newNode;
}

// Create graph of vertices V and declares adjacency list
struct Graph* createGraph(int V)
{
    struct Graph* graph = new Graph;
    graph->V = V;
    // Create an array of adjacency lists.  Size of array will be V
    graph->array = new AdjList[V+1];
    graph->edgeCount = new int[V+1];
    graph->E = 0;
    
     // Initialize each adjacency list as empty by making head as NULL
    int i;
    for (i = 1; i <= V; ++i)
    {
        graph->array[i].head = NULL;
        graph->edgeCount[i] = 0;
    }
    return graph;
}

// Adds an edge to an undirected graph
void addEdge(struct Graph* graph, int src, int dest)
{
    graph->E = graph->E + 1;
    // Add an edge from source to destination. A new node is added to the adjacency
    // list of src.  The node is added at the head
    int tempWeight = rand()%10000 + 1; // add 1 to set the weight range between 1 and 1000
    struct AdjListNode* newNode = newAdjListNode(dest);
    newNode->next = graph->array[src].head;
    newNode->weight=tempWeight;
    graph->array[src].head = newNode;
    graph->edgeCount[src] = graph->edgeCount[src] + 1;
 
    // Since graph is undirected
    newNode = newAdjListNode(src);
    newNode->next = graph->array[dest].head;
    newNode->weight=tempWeight;
    graph->array[dest].head = newNode;
    graph->edgeCount[dest] = graph->edgeCount[dest] + 1;
}

// Print the adjacenncy list representation of graph
void printGraph(struct Graph* graph)
{
    int v;
    for (v = 1; v <= graph->V; ++v)
    {
        struct AdjListNode* pCrawl = graph->array[v].head;
        cout<<"\n Adjacency list of vertex "<< v <<" with "<<graph->edgeCount[v]<<" edges"<< "\n head ";
        
        while (pCrawl)
        {
            //cout<<"-> "<< pCrawl->dest<<"("<<pCrawl->weight<<")";
            pCrawl = pCrawl->next;
        }
        
        cout<<endl;
    }
}

//Creating graph G2
struct Graph* createG1()
{
    int V = 5000;
    bool (*adjMat)[5001];
    adjMat = new bool[5001][5001];
    
    struct Graph* graph = new Graph;
    graph->V = V;
    // Create an array of adjacency lists.  Size of array will be V
    graph->array = new AdjList[V+1];
    graph->edgeCount = new int[V+1];
    
     // Initialize each adjacency list as empty by making head as NULL
    int i, j;
    for (i = 0; i <= V; ++i)
    {
        graph->array[i].head = NULL;
        graph->edgeCount[i] = 0;
        for(j=0;j<=V;j++)
            adjMat[i][j]=false;
    }
    
    //Populating graph with random edges -- degree of a vertex = 8
    
    //Cycle edges
    for(i = 1 ; i<V ; i++)
    {
        addEdge(graph, i , i+1);
        adjMat[i][i+1] = true;
        adjMat[i+1][i] = true;
    }
    addEdge(graph, i , 1);
    adjMat[i][1] = true;
    adjMat[1][i] = true;
    i=1;
    
    for(int s = 1;s<=5000;s++)
    {
        for(int t = s+2;t<=5000;t++)
        {
            int p = rand()%5000;
            if(p<6 && !(adjMat[s][t]))
            {
            addEdge(graph,s,t);
            adjMat[s][t] = true;
            adjMat[t][s] = true;
            }
        }
    }
    return graph;
}


//Creating graph G2
struct Graph* createG2()
{
    int V = 5000;
    bool (*adjMat)[5001];
    adjMat = new bool[5001][5001];
    
    struct Graph* graph = new Graph;
    graph->V = V;
    // Create an array of adjacency lists.  Size of array will be V
    graph->array = new AdjList[V+1];
    graph->edgeCount = new int[V+1];
    
     // Initialize each adjacency list as empty by making head as NULL
    int i, j;
    for (i = 0; i <= V; ++i)
    {
        graph->array[i].head = NULL;
        graph->edgeCount[i] = 0;
        for(j=0;j<=V;j++)
            adjMat[i][j]=false;
    }
    
    //Populating graph with random edges -- degree of a vertex = 8
    
    //Cycle edges
    for(i = 1 ; i<V ; i++)
    {
        addEdge(graph, i , i+1);
        adjMat[i][i+1] = true;
        adjMat[i+1][i] = true;
    }
    addEdge(graph, i , 1);
    adjMat[i][1] = true;
    adjMat[1][i] = true;
    i=1;
    //select random edges
    
    for(int s = 1;s<=5000;s++)
    {
        for(int t = s+2;t<=5000;t++)
        {
            int p = rand()%100;
            if(p<20 && !(adjMat[s][t]))
            {
            addEdge(graph,s,t);
            adjMat[s][t] = true;
            adjMat[t][s] = true;
            }
        }
    }
    return graph;
}


//--------------------------------------DFS---------------------------

void DFSUtil(struct Graph* graph, int s, int t, bool visited[], int *parent,int* maxBand)
{
    
    // Mark the current node as visited and print it
    visited[s] = true;
    //cout << s << " ";
 
    // Recur for all the vertices adjacent to this vertex
    struct AdjListNode* pcrawl = graph->array[s].head;
    while(pcrawl)
    {
        if(visited[t]==true)
        {
            return;
        }   
        if(!(visited[pcrawl->dest]))
        {
            parent[pcrawl->dest] = s;
            maxBand[pcrawl->dest] = maxBand[s]<(pcrawl->weight)?maxBand[s]:(pcrawl->weight);
            if(pcrawl->weight == 0)
                cout<<pcrawl->dest<<"---"<<pcrawl->weight<<endl;
            DFSUtil(graph,pcrawl->dest,t, visited,parent,maxBand);
        }
        pcrawl = pcrawl->next;
    }
}

void DFS(struct Graph* graph,int s, int t,int *parent)
{
    // Mark all the vertices as not visited
    bool *visited = new bool[graph->V + 1];
    //int *parent = new int[graph->V + 1];
    for (int i = 0; i <= graph->V; i++)
        visited[i] = false;
    int* maxBand = new int[graph->V + 1];
    maxBand[s] = 20000;
 
    // Call the recursive helper function to print DFS traversal
    // starting from all vertices one by one
    DFSUtil(graph, s, t, visited,parent,maxBand);
    
    
    cout<<endl<<endl<<"****Maximum Bandwidth based on modification of Kruskal****";
    cout<<endl<<"Maximum Bandwidth="<<maxBand[t];
     cout<<endl<<"Maximum bandwidth path from vertex "<<s<<" to vertex "<<t<<" :";
    int i = t;
    cout<<endl<<i;;
    while(i!=s)
    {
        i = parent[i];
        cout<<" <-- "<<i;
    }
}
//--------------------------------------DFS ends---------------------------


//----------------------------Kruskal Begins---------------------------------
// A structure to represent a subset for union-find
struct subset
{
    int parent;
    int rank;
};
 
// A utility function to find set of an element i
// (uses path compression technique)
int find(struct subset subsets[], int i)
{
    // find root and make root as parent of i 
    // (path compression)
    if (subsets[i].parent != i)
    {
        subsets[i].parent = find(subsets, subsets[i].parent);
    }
    return subsets[i].parent;
}
 
// A function that does union of two sets of x and y
// (uses union by rank)
void Union(struct subset subsets[], int x, int y)
{
    int xroot = find(subsets, x);
    int yroot = find(subsets, y);
 
    // Attach smaller rank tree under root of high 
    // rank tree (Union by Rank)
    if (subsets[xroot].rank < subsets[yroot].rank)
        subsets[xroot].parent = yroot;
    else if (subsets[xroot].rank > subsets[yroot].rank)
        subsets[yroot].parent = xroot;
 
    // If ranks are same, then make one as root and 
    // increment its rank by one
    else
    {
        subsets[yroot].parent = xroot;
        subsets[xroot].rank++;
    }
}

struct Graph* KruskalMST(struct Graph* graph)
{
    int V = graph->V;
    struct Graph* mst = new Graph;//createGraph(V);  // Tnis will store the resultant MST
    
    mst->V = graph->V;
    mst->array = new AdjList[graph->V + 1];
    mst->edgeCount = new int[graph->V + 1];
    mst->E = 0;
    
    int e = 0;  
    int i = 0; 
 
 //---------------------------------------
     int *H = new int[(graph->E)+1];
        struct Node *node = new Node[graph->E + 1];
        int* I = new int[graph->E + 1];//Index of vertices
        for(int i = 0;i<(graph->E + 1);i++)
            node[i].weight = -1;
        int length = 0;
        int tracker = 1;
        
        
        for (int v = 1; v <= graph->V; ++v)
        {
            struct AdjListNode* pCrawl = graph->array[v].head;
           
            while (pCrawl)
            {
                if(v<(pCrawl->dest))
                {
                    H[tracker] = tracker;
                    node[H[tracker]].src = v;
                    node[H[tracker]].dest = pCrawl->dest;
                    insert_value(H,node,I,tracker,pCrawl->weight,&length);
                    tracker++;
                }
                pCrawl = pCrawl->next;
            }
        }
        
 //---------------------------------------
    // Allocate memory for creating V subsets
    struct subset *subsets = new subset[V+1];
 
    // Create V subsets with single elements
    for (int v = 1; v <= V; ++v)
    {
        subsets[v].parent = v;
        subsets[v].rank = 0;
    }
 
    // Number of edges to be taken is equal to V-1
    while (e < V - 1)
    {
        // Step 2: Pick the smallest edge. And increment 
        // the index for next iteration
        i = extract_maximum (H , node, I, &length);
        int x = find(subsets, node[i].src);
        int y = find(subsets, node[i].dest);
 
        // If including this edge does't cause cycle,
        // include it in result and increment the index 
        // of result for next edge
        if (x != y)
        {
            int src = node[i].src;
            int dest = node[i].dest;
            int tempWeight = node[i].weight; // add 1 to set the weight range between 1 and 1000
            struct AdjListNode* newNode = newAdjListNode(dest);
            newNode->next = mst->array[src].head;
            newNode->weight=tempWeight;
            mst->array[src].head = newNode;
            mst->edgeCount[src] = mst->edgeCount[src] + 1;
         
            // Since graph is undirected
            newNode = newAdjListNode(src);
            newNode->next = mst->array[dest].head;
            newNode->weight=tempWeight;
            mst->array[dest].head = newNode;
            mst->edgeCount[dest] = mst->edgeCount[dest] + 1;
            mst->E = mst->E + 1;
            
            Union(subsets, x, y);
            e++;
        }
    }
    return mst;
}
//----------------------------Kruskal Ends---------------------------------




//Using Dijkstra without heaps
int* maxBandwidthPath1(Graph* graph, int s, int t)
{
    int i;
    struct AdjListNode* pcrawl;
    int (*parent) = new int[(graph->V)+1];
    int v;//fringe being processed
    char (*status) = new char[(graph->V)+1];
    
    int D[5001];
    for(i = 0;i<5001;i++)
        D[i] = -1;
    
    
    for(i=1;i<=graph->V;i++)
    {
        status[i] = 'u';//unseen
    }
    
    status[s] = 'i';//intree
    D[s] = 20000;//bandwidth ranges between 0 and 1000 -- 10k is relatively infinite//-----bw to D
    
    pcrawl = graph->array[s].head;
    while(pcrawl)
    {
        status[pcrawl->dest] = 'f';//fringe
        D[pcrawl->dest] = pcrawl->weight;
        parent[pcrawl->dest] = s;
        pcrawl = pcrawl->next;
    }
    
    while(status[t]!='i')
    {
        v = -1;int maxF=-1;
        for(i = 1;i<=5000;i++)
        {
            if((status[i]=='f') && (maxF < D[i]))
            {
                v = i;
                maxF = D[i];
            }
        }
        if(v==-1)
            continue;
        status[v] = 'i';
        pcrawl = graph->array[v].head;
        while(pcrawl)
        {
            int min = D[v]<(pcrawl->weight)?D[v]:(pcrawl->weight);
            if(status[pcrawl->dest] == 'u')
            {
                status[pcrawl->dest] = 'f';
                D[pcrawl->dest] = min;
                parent[pcrawl->dest] = v;
            }
            else if((status[pcrawl->dest] == 'f') && (D[pcrawl->dest] < min))//-----bw to D
            {
                D[pcrawl->dest] = min;
                if(D[pcrawl->dest]>10000)
                    cout<<"Pakda gaya--"<<endl;
                parent[pcrawl->dest] = v;
            }
            pcrawl = pcrawl->next;
        }
    }
    cout<<endl<<endl<<"****Maximum Bandwidth using Dikjstra without heap****";
    cout<<endl<<"Maximum bandwidth = "<<D[t];//-----bw to D
     cout<<endl<<"Maximum bandwidth path from vertex "<<s<<" to vertex "<<t<<" :";
    i = t;
    cout<<endl<<i;;
    while(i!=s)
    {
        i = parent[i];
        cout<<" <-- "<<i;
    }
    delete[] status;
    return parent;
}


//Using Dijkstra with heaps
int* maxBandwidthPath2(Graph* graph, int s, int t)
{
    int i;
    struct AdjListNode* pcrawl;
    //int (*bw) = new int[(graph->V)+1];
    int (*parent) = new int[(graph->V)+1];
    int v;//fringe being processed
    char (*status) = new char[(graph->V)+1];
    
    //Variable used for heap implementation
    int H[5001];
    struct Node *node = new Node[graph->V + 1];
    int I[graph->V + 1];//Index of vertices
    for(i = 0;i<(graph->V + 1);i++)
        node[i].weight = -1;
    int length = 0;
    
    
    for(i=1;i<=graph->V;i++)
    {
        status[i] = 'u';//unseen
    }
    
    status[s] = 'i';//intree
    node[s].weight = 20000;//bandwidth ranges between 0 and 1000 -- 10k is relatively infinite//-----bw to D
    
    pcrawl = graph->array[s].head;
    while(pcrawl)
    {
        status[pcrawl->dest] = 'f';//fringe
        insert_value(H,node,I,pcrawl->dest,pcrawl->weight,&length);//insert fringe to the heap
        parent[pcrawl->dest] = s;
        pcrawl = pcrawl->next;
    }
    
    while(status[t]!='i')
    {
        v = extract_maximum(H,node,I,&length);//not implemented yet
        //cout<<v<<endl;
        if(v==-1)
            continue;
        //cout<<v<<endl;
        status[v] = 'i';
        pcrawl = graph->array[v].head;
        while(pcrawl)
        {
            //int min = bw[v]<(pcrawl->weight)?bw[v]:(pcrawl->weight);
            int min = node[v].weight<(pcrawl->weight)?node[v].weight:(pcrawl->weight);//-----bw to D
            if(status[pcrawl->dest] == 'u')
            {
                status[pcrawl->dest] = 'f';
                insert_value(H,node,I,pcrawl->dest,min,&length);//insert fringe to the heap
                parent[pcrawl->dest] = v;
            }
            else if((status[pcrawl->dest] == 'f') && (node[pcrawl->dest].weight < min))//-----bw to D
            {
                increase_value(H,node,I,pcrawl->dest,min,length);
                if(node[pcrawl->dest].weight>10000)
                    cout<<"Pakda gaya--"<<endl;
                parent[pcrawl->dest] = v;
            }
            pcrawl = pcrawl->next;
        }
    }
    
    cout<<endl<<endl<<"****Maximum Bandwidth using Dikjstra with heap****";
    cout<<endl<<"Maximum bandwidth="<<node[t].weight;//-----bw to D
     cout<<endl<<"Maximum bandwidth path from vertex "<<s<<" to vertex "<<t<<" :";
    i = t;
    cout<<endl<<i;;
    while(i!=s)
    {
        i = parent[i];
        cout<<" <-- "<<i;
    }
    delete[] status;
    return parent;
}

//Using Kruskal to fing maximum bandwidth
struct Graph* maxBandwidthPath3(Graph* graph, int s, int t)
{
    
    int (*parent) = new int[(graph->V)+1];
    
    struct Graph* mst = KruskalMST(graph);
    DFS(mst,s,t,parent);
    
    if(parent[t] == 0)
    {    
        AdjListNode* pcrawl = graph->array[t].head;
        while(pcrawl)
        {
            cout<<pcrawl->dest<<"~~~~";
            pcrawl=pcrawl->next;
        }
    }
    return mst;
}

int main()
{
    // create the graph given in above fugure
    srand((unsigned)time(NULL));
    //int V = 20;
    struct Graph* graph;// = createG1();
    
    //If you want to find the average degree of a graph, de-comment this section
    /*
    struct Graph* graph;// = createG1();
    
    double average=0;
    
    cout<<endl<<"--------------------Sparse Graph------------------------------";
    for(int j = 1;j<=10;j++)
    {
        graph = createG1();
        average = 0;
        for(int i = 1;i<=5000;i++)
            average = average + graph->edgeCount[i];
        
        average = average/5000;
        cout<<endl<<"Average:"<<average;
    }
    
    cout<<endl<<"--------------------Dense Graph------------------------------";
    
    for(int j = 1;j<=10;j++)
    {
        graph = createG2();
        average = 0;
        for(int i = 1;i<=5000;i++)
            average = average + graph->edgeCount[i];
        
        average = average/5000;
        cout<<endl<<"Average:"<<average;
    }
    */
    
    //If you want to print the graph generated use de-comment the next line
    //printGraph(graph);
    
    
    int compMat1[11][5];
    int compMat2[11][5];
    int compMat3[11][5];
    
    int noOfIterations = 1;
    
    clock_t ts1,ts2;
    for(int i=0;i<noOfIterations;i++)
    {
        graph = createG2(); //----------------------Change here to createG1() to change the Graph type to G1
        struct Graph* mst;
        for(int j=0;j<5;j++)
        {
            int s = rand()%5000 + 1;
            int t = rand()%5000 + 1;
            ts1 = clock();
            maxBandwidthPath1(graph,s,t);
            ts2 = clock();
            compMat1[i][j] = ts2-ts1;
            
            ts1 = clock();
            maxBandwidthPath2(graph,s,t);
            ts2 = clock();
            compMat2[i][j] = ts2-ts1;
            
            
            ts1 = clock();
            if(j==0)
            {
                mst = maxBandwidthPath3(graph,s,t);
            }
            else
            {
                int (*parent) = new int[(graph->V)+1];
                DFS(mst,s,t,parent);
            }
            ts2 = clock();
            compMat3[i][j] = ts2-ts1;
        }
    }
    
    cout<<endl<<"------------------------------------------------------------------"<<endl;
    double average = 0;
    cout<<"Time taken in clock ticks by : Maximum Bandwidth Path based on Dikjstra without heap"<<endl;
    for(int i = 0;i<noOfIterations;i++)
    {
        for(int j = 0;j<5;j++)
        {
            cout<<setw(10)<<compMat1[i][j];
            average = average + compMat1[i][j];
        }
        cout<<endl;
    }
    cout<<"average = "<<(int)(average/25)<<endl; 
    cout<<"-------------"<<endl;
    average = 0;
    cout<<"Time taken in clock ticks by : Maximum Bandwidth Path based on Dikjstra with heap"<<endl;
    for(int i = 0;i<noOfIterations;i++)
    {
        for(int j = 0;j<5;j++)
        {
            cout<<setw(10)<<compMat2[i][j];
            average = average + compMat2[i][j];
        }
        cout<<endl;
    }
    cout<<"average = "<<(int)(average/25)<<endl;
    cout<<"-------------"<<endl;
    average = 0;
    cout<<"Time taken in clock ticks by : Maximum Bandwidth Path based on Modification of Kruskal"<<endl;
    for(int i = 0;i<noOfIterations;i++)
    {
        for(int j = 0;j<5;j++)
        {
            cout<<setw(10)<<compMat3[i][j];
            average = average + compMat3[i][j];
        }
        cout<<endl;
    }
    cout<<"average = "<<(int)(average/25)<<endl;
    
    return 0;
}