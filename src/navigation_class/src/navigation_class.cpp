//Developed for "ARS" by Pinchuk P.I. 16.05.2018 date. v.1.008

#include "../include/navigation_class.hpp"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cstring>
#include <utility>
#include <deque>
#include <climits>
#include <algorithm>
#include <sstream>

using namespace std;

#define MAX 1000000000 // infinity definition
#define STRING_LENGTH 100 // max string length definition
//string sourceFile = "matrix.txt"
std::vector<std::string> navigation_class::optGraphMarshrut( std::string sSp, std::string sFp)
{
    int st = 0 ; int fp =0;
    char* s1[40];

    vector<std::string> vGraphPointStr, vPathStr;
    int weightEdge, selfclosingedge = 0,numberOfVertices = 3;
    std::string pointname;
    if (sSp == sFp) selfclosingedge = 1;
    std::ifstream sourseFile("matrix.txt");
    vector < vector< pair< int, int> > > vvGraph(numberOfVertices);
    sourseFile >> numberOfVertices;
    vvGraph.resize (numberOfVertices);

    std::string s_file = "matrix.txt";
    std::ifstream t_file;
    t_file.open(s_file);
    std::stringstream s_string_stream;

    s_string_stream << t_file.rdbuf();

    std::string tok_;

    getline(s_string_stream, tok_, ' ');

        int MAX_LENGTH = STRING_LENGTH;
    char line[MAX_LENGTH];
    for (int i = 0; i <= numberOfVertices; i++) {
        sourseFile.getline(line, MAX_LENGTH);
        vGraphPointStr.push_back(line);

        if (tok_== sSp){ st = i; }
        if (tok_== sFp){ fp = i; }
    }
    int startVertex = st-1, finishVertex = fp-1;

    for (int i = 0; i < numberOfVertices; i++) {       // Get adjacency matrix from sourse file.
        for (int j = 0; j < numberOfVertices; j++) {
            sourseFile >> weightEdge;
            vvGraph[i].push_back (make_pair(j, weightEdge));  //adding an oriented edge from i to j with weightEdge weight
        }
    }
    if (selfclosingedge ==1){
        numberOfVertices =numberOfVertices+1;
        vvGraph.resize (numberOfVertices);
        for (int i = 0; i < numberOfVertices-1; i++) {
            vvGraph[i].push_back(make_pair(numberOfVertices-1, vvGraph[i][st-1].second));  //selfclosing edge treatment
        }
        vvGraph[numberOfVertices].push_back(make_pair(numberOfVertices, MAX));
        for (int j = 0; j < numberOfVertices; j++) {
            vvGraph[numberOfVertices].push_back(make_pair(j, MAX));
        }
            finishVertex = numberOfVertices-1;
    }
    vector<int> vDigitalGraph (numberOfVertices, MAX), state (numberOfVertices, 2), vPathGraph (numberOfVertices, -1);
    deque<int> vQuantity;
    vDigitalGraph[startVertex] = 0;
    state[startVertex] = 1;
    vQuantity.push_back (startVertex);
    while (!vQuantity.empty())
    {
        int vertex = vQuantity.front();
        vQuantity.pop_front(), state[vertex] = 0;
        for (size_t i = 0; i < vvGraph[vertex].size(); ++i)
        {
            int to = vvGraph[vertex][i].first, length = vvGraph[vertex][i].second;
            if (vDigitalGraph[to] > vDigitalGraph[vertex] + length)
            {
                vDigitalGraph[to] = vDigitalGraph[vertex] + length;
                if (state[to] == 2) vQuantity.push_back (to);
                else if (state[to] == 0)
                    vQuantity.push_front (to);
                vPathGraph[to] = vertex;
                state[to] = 1;
            }
        }
    }
    if (vPathGraph[finishVertex] == -1)
    {
        vPathStr.push_back("No solutions");
    }
    else
    {
        vector<int> path;
        for (int vertex = finishVertex; vertex != -1; vertex = vPathGraph[vertex]) path.push_back (vertex);
        reverse (path.begin(), path.end());
        if (selfclosingedge ==1) path[path.size()-1] =st-1;
        for (int i = 0; i < path.size(); i++){
            vPathStr.push_back(vGraphPointStr[path[i]+1]);
          //  cout << path[i] << endl;
        }
        return vPathStr;
    }
}