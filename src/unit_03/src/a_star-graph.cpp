#include <cstdio>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include<list>
#include<iterator>
#include <functional>
#include <queue>
#include <vector>
#include <string_view>
#include <math.h>

using namespace std;
class Graph_Navigator{
    public:
        Graph_Navigator();
        void mainLoop();

    protected:
    // Nodehandle for Just_move robot
    ros::NodeHandle m_nodeHandle;
   
    private:
        struct knoten{
            bool visited = false;
            double f;
            double g;
            double x;
            double y;
            vector<knoten*> adjacent;
            knoten* pred;
        };
        double** readFile(char* fileName, int length);
        double euklid(knoten* a, knoten* b);
        double heuristik(knoten* a, knoten* b);
        void a_star(knoten* graph, int start, int end);
};

// constructor
Graph_Navigator::Graph_Navigator() {

    // Node will be instantiated in root-namespace
    ros::NodeHandle m_nodeHandle("/");

}// end of Just_move constructor
    
double Graph_Navigator::euklid(knoten* a, knoten* b){
    return sqrtf(pow(a->x - b->x,2) + pow(a->y - b->y,2));
}
    
double Graph_Navigator::heuristik(knoten* a, knoten* b){
     return euklid(a,b);
}
    
void Graph_Navigator::a_star(knoten* graph, int start, int end){
    knoten *current, *startKnoten, *endKnoten;
    startKnoten = &graph[start];
    endKnoten = &graph[end];
    startKnoten->g = 0.0;
    startKnoten->f = heuristik(startKnoten, endKnoten);
        
    list<knoten*> openList;
    openList.push_back(startKnoten);
        
    while(!openList.empty()){
        openList.sort([](const knoten* lhs, const knoten* rhs){return lhs->f < rhs->f;});
            
        while(!openList.empty()&&openList.front()->visited){
            openList.pop_front();
        }
        if(openList.empty()){break;}
            
        current = openList.front();
        current->visited = true;
            
        for(auto nachbarKnoten : current->adjacent){
            if(!nachbarKnoten->visited){
                openList.push_back(nachbarKnoten);
            }
            double temp = current->g + euklid(current,nachbarKnoten);
                
            if(temp < nachbarKnoten->g){
                nachbarKnoten->pred = current;
                nachbarKnoten->g = temp;
                nachbarKnoten->f = nachbarKnoten->g + heuristik(nachbarKnoten,endKnoten);
                //printf("%lf    %lf  %lf\n",nachbarKnoten->pred->x,nachbarKnoten->pred->y,heuristik(nachbarKnoten,endKnoten));
            }
        }
    }
}

double**Graph_Navigator::readFile(char* fileName, int length){

    FILE *fp;
    double i,l,k;                                   
    int j;
    

    fp = fopen (fileName, "r");
    if(fp!=NULL){printf("File opened\n");}

    double **arr=(double**)malloc(sizeof(double*)*length);                         
    for(j=0;j<length;j++){
        arr[j]=(double*)malloc(3*sizeof(double));
    }

    j=0;
    while(fscanf(fp, "%lf\t%lf\t%lf", &i,&l,&k)==3){
        arr[j][0]=i;
        arr[j][1]=l;                                   
        arr[j][2]=k;
        j++;
        if(j==length){break;}                                   
    }
    printf("Success\n");
    fclose(fp);
    
    if(j==0){ROS_INFO("File could not be read");}
   
    return arr;
}

void Graph_Navigator::mainLoop(){
    // determines the number of loops per second
    ros::Rate loop_rate(20);

    //read files
    double **kn,**ka;
    knoten *graph = nullptr;
    knoten *startKnoten, *endKnoten, *pfadKnoten = nullptr;
    char name1[]={"/home/user/ropra/PG_home/WS22_ws/src/unit_03/kanten_prakt_02.txt"};
    char name2[]={"/home/user/ropra/PG_home/WS22_ws/src/unit_03/knoten_prakt_02.txt"};

    ka=readFile(name1,98);
    kn=readFile(name2,60);

    graph = new knoten[60];
            
    for(int i=0;i<60;i++){
        graph[i].visited = false;
        graph[i].f = INFINITY;
        graph[i].g = INFINITY;
        graph[i].x = kn[i][1];
        graph[i].y = kn[i][2];
        graph[i].pred = nullptr;
    }
            
    //printf("Kanten einfuegen\n");
    for(int i=0;i<98;i++){
        graph[(int)ka[i][1]].adjacent.push_back(&graph[(int)ka[i][2]]);
        //printf("%d  %d  %d\n",(int)ka[i][1],(int)ka[i][2],i);
    }

    a_star(graph, 0, 59);
            
    startKnoten = &graph[0];
    endKnoten = &graph[59];
    list<knoten*> pfad;
    pfadKnoten = endKnoten;
        
    while(pfadKnoten->pred!=nullptr){
        //printf("%lf    %lf\n",pfadKnoten->x,pfadKnoten->y);
        pfad.push_front(pfadKnoten);
        pfadKnoten = pfadKnoten->pred;
    }
    pfad.push_front(startKnoten);

    while(!pfad.empty()){
        printf("%lf    %lf\n",pfad.front()->x,pfad.front()->y);
        pfad.pop_front();
    }

    // als long as all is O.K : run
    // terminate if the node get a kill signal
    while (m_nodeHandle.ok()){
        // spinOnce, just make the loop happen once
        ros::spinOnce();
        // and sleep, to be aligned with the 50ms loop rate
        loop_rate.sleep();

    }// end of if nodehandle O.K.

}// end of mainLoop


int main(int argc, char** argv){

    // initialize
    ros::init(argc, argv, "Graph_Navigator");

    // get an object of type Graph_Navigator and call it robbi
    Graph_Navigator robbi;

    // make robbi run
    // main loop
    robbi.mainLoop();

    return 0;
}