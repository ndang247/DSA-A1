#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <queue>
#include <stack>
#include <limits>
#include <utility> 
#include <algorithm>
#include <string>
#include <cstdlib>

#include <iterator>
#include <map>
#include <climits>

using namespace std;

template <typename T> // the template allows the weight of an edge to take any numeric data type (denoted by T).
class Graph {

	public:
        
        /* define your data structure to represent a weighted undirected graph */

        // key vertex A : value map<string, T> { {B, 7}, {C, 2} }
        map< string, map<string, T> > adj_list_weighted_edges;
                    
        /* test1 */
		Graph(); // the contructor function.
		~Graph(); // the destructor function.
		size_t num_vertices(); // returns the total number of vertices in the graph.
		size_t num_edges(); // returns the total number of edges in the graph.

        /* test2 */
        void add_vertex(const string&); // adds a vertex to the graph -- every vertex uses a string as its unique identifier.
        bool contains(const string&); // checks if a vertex is in the graph -- returns true if the graph contains the given vertex; otherwise, returns false.
        
        /* test3 */
        vector<string> get_vertices(); // returns a vector of all the vertices in the graph.

        /* test4 */
        void add_edge(const string&, const string&, const T&); // adds a weighted edge to the graph -- the two strings represent the incident vertices; the third parameter represents the edge's weight.
        bool adjacent(const string&, const string&); // check if there is an edge between the two vertices in the graph -- returns true if the edge exists; otherwise, returns false.
		
        /* test5 */
        vector<pair<string,string>> get_edges(); // returns a vector of all the edges in the graph -- each edge is represented by a pair of vertices incident to the edge.
        
        /* test6 */
        vector<string> get_neighbours(const string&); // returns a vector of all the vertices, each of which is directly connected with the given vertex by an edge.
        size_t degree(const string&); // returns the degree of a vertex.

        /* test7 */
		void remove_edge(const string&, const string&); // removes the edge between two vertices, if it exists.
        
        /* test8 */
        void remove_vertex(const string&); // delete the given vertex from the graph -- note that, all incident edges of the vertex should be deleted as well.

        /* test9 */
		vector<string> depth_first_traversal(const string&); // returns a vector of all the vertices in the visiting order of a depth-first traversal from the given vertex.
		
        /* test10 */
        vector<string> breadth_first_traversal(const string&); // returns a vector of all the vertices in the visiting order of a breadth-first traversal from the given vertex.
        
        /* test11 */
		bool contain_cycles(); // check if the graph contains any cycle -- returns true if there exists a path from a vertex to itself; otherwise, return false.
        
        /* test12 */
		Graph<T> minimum_spanning_tree(); // returns a spanning tree of the graph -- the returned tree is preferably a minimum spanning tree.

        bool has_cycle(string, set<string>&, string); // returns true if a cycle has found.

        string min_adj_vertex(map<string, T>&, vector<string>&); // a helper function to find the adj vertex with minimum weight or key from the set of vertices has not visited. 
};

/* test1 */

template <typename T>
Graph<T>::Graph() {}

template <typename T>
Graph<T>::~Graph() {}


template <typename T>
size_t Graph<T>::num_vertices() {
    return adj_list_weighted_edges.size();
}

template <typename T>
size_t Graph<T>::num_edges() {
    size_t num_edges = 0;
    for (auto itr = adj_list_weighted_edges.begin(); itr != adj_list_weighted_edges.end(); ++itr) { 
        num_edges += itr->second.size();
    }
    return (num_edges / 2);
}

/* test2 */

template <typename T>
void Graph<T>::add_vertex(const string& u) {
    adj_list_weighted_edges.insert(make_pair(u, map<string, T>()));
}

template <typename T>
bool Graph<T>::contains(const string& u) {
    // true if the graph contains u
    // return !(adj_list_weighted_edges.find(u) == adj_list_weighted_edges.end());
    return adj_list_weighted_edges.count(u);
}

/* test3 */

template <typename T>
vector<string> Graph<T>::get_vertices() {
    vector<string> vertices;
    for (auto itr = adj_list_weighted_edges.begin(); itr != adj_list_weighted_edges.end(); ++itr) {
        vertices.push_back(itr->first);
    }
    return vertices;
}

/* test4 */

template <typename T>
void Graph<T>::add_edge(const string& u, const string& v, const T& weight) {
    if (contains(u) && contains(v)) {
        // pair<string, T> adj_vertex_1 (v, weight);
        // pair<string, T> adj_vertex_2 (u, weight);
        // auto var_1 = adj_list_weighted_edges.find(u);
        // auto var_2 = adj_list_weighted_edges.find(v);
        // var_1->second.insert(adj_vertex_1);
        // var_2->second.insert(adj_vertex_2);
        adj_list_weighted_edges[u].insert(make_pair(v , weight));
        adj_list_weighted_edges[v].insert(make_pair(u , weight));
    }
}

template <typename T>
bool Graph<T>::adjacent(const string& u, const string& v) {
    if (contains(u) && contains(v)) {
        // auto var = adj_list_weighted_edges.find(u);
        // return true if the second map contains v
        // return !(var->second.find(v) == var->second.end()); // true if v not in the second map
        return adj_list_weighted_edges[u].count(v);
    }
    else {
        return false;
    }
}

/* test5 */

template <typename T>
vector<pair<string,string>> Graph<T>::get_edges() {
    vector<pair<string, string>> edges;

    for (auto itr = adj_list_weighted_edges.begin(); itr != adj_list_weighted_edges.end(); ++itr) {
        for (auto itr_adj_vertex = itr->second.begin(); itr_adj_vertex != itr->second.end(); ++itr_adj_vertex) {
            pair<string, string> edge_1 (itr->first, itr_adj_vertex->first);
            pair<string, string> edge_2 (itr_adj_vertex->first, itr->first);
            
            // only add an edge if edge_1 or edge_2 is not in the vector edges
            if ( !( find(edges.begin(), edges.end(), edge_1) != edges.end() || 
                    find(edges.begin(), edges.end(), edge_2) != edges.end() ) ) {
                edges.push_back(edge_1);
            }
        }
    }
    return edges;
}

/* test6 */

template <typename T>
vector<string> Graph<T>::get_neighbours(const string& u) {
    vector<string> neighbours;
    for (auto itr = adj_list_weighted_edges[u].begin(); itr != adj_list_weighted_edges[u].end(); ++itr) {
        neighbours.push_back(itr->first);
    }
    return neighbours;
}

template <typename T>
size_t Graph<T>::degree(const string& u) {
    return get_neighbours(u).size();
}

/* test7 */

template <typename T>
void Graph<T>::remove_edge(const string& u, const string& v) {
    // auto var = adj_list_weighted_edges.find(u);
    // if (!(var->second.find(v) == var->second.end())) {
    //     var->second.erase(v);
    //     remove_edge(v, u);
    // }
    if (adj_list_weighted_edges[u].count(v)) {
        adj_list_weighted_edges[u].erase(v);
        remove_edge(v, u);
    }
}

/* test8 */

template <typename T>
void Graph<T>::remove_vertex(const string& u) {
    auto neighbours = get_neighbours(u);
    for (auto neighbour : neighbours) {
        remove_edge(neighbour, u);
    }
    adj_list_weighted_edges.erase(u);
}

/* test9 */

template <typename T>
vector<string> Graph<T>::depth_first_traversal(const string& u) {
    set<string> visited; // store vertices have been visited

    stack<string> s; // store the list of next visited vertices
    s.push(u);
    vector<string> paths;

    while (!s.empty()) {
        string v = s.top();
        s.pop();
        if (!(visited.find(v) != visited.end())) { // if visited does not contain v
            visited.insert(v);
            paths.push_back(v); 
            for (int i = get_neighbours(v).size(); i != 0; i--) {
                s.push(get_neighbours(v)[i - 1]);
            }
            // OR
            // for (int i = 0; i < get_neighbours(v).size(); i++) {
            //     s.push(get_neighbours(v)[i]);
            // }
        }
    }
    return paths;
}

/* test10 */

template <typename T>
vector<string> Graph<T>::breadth_first_traversal(const string& u) {
    set<string> visited;
    
    queue<string> q;
    q.push(u);
    vector<string> paths;

    while (!q.empty()) {
        string v = q.front();
        q.pop();
        if (!(visited.find(v) != visited.end())) {
            visited.insert(v);
            paths.push_back(v);
            for (int i = 0; i < get_neighbours(v).size(); i++) {
                q.push(get_neighbours(v)[i]);
            }
        }
    }
    return paths;
}

/* test11 */

template <typename T>
bool Graph<T>::contain_cycles() {
    set<string> visited; // a set that store all the visited vertex
    for (string vertex : get_vertices()) {
        if (!(visited.find(vertex) != visited.end())) { // if the vertex is not already in visited
            if (has_cycle(vertex, visited, "-")) { // dfs a vertex and if true contains a cycle, let the parent here a random string since the starting vertex has no parent
            return true;
            }
        }
    }
    return false;
    // An alternative way to check for cycles but only works for connected graph 
    // return num_edges() >= num_vertices(); // true means there is a cycle
}

/* test12 */

template <typename T>
Graph<T> Graph<T>::minimum_spanning_tree() {
    map<string, T> key; // store key value of vertices -> {A:0, B:7, ...}
    vector<string> mstSet; // store a set of all vertices included in MST {A, B, C, ...}
    map<string, string> parent; // store parent vertices in MST for example {B:A, ...} where A is parent of B
    Graph<int> min_spanning_tree;

    for (auto itr = adj_list_weighted_edges.begin(); itr != adj_list_weighted_edges.end(); ++itr) {
        key.insert(make_pair(itr->first, INT_MAX)); // set all the key of all vertices to the max value or INF
        parent.insert(make_pair(itr->first, "np")); // set all the parent of all vertices to np or no parent
    }

    key.begin()->second = 0; // first element has 0 key
    parent.begin()->second = "-1"; // first element has no parent

    while (mstSet.size() < num_vertices()) {
        // once updated the parent and key, find the vertex with smallest key in the key map
        string u = min_adj_vertex(key, mstSet);

        mstSet.push_back(u); // add the min vertex (vertex that has the min value in the key map) as visited in mstSet 

        // go through the neighbour of the min vertex (u) which is the second map of adj_list_weighted_edges
        // if the neighbour has not been visited and its key (weight) is less than the current value in the key map
        // then update the parent of the neighbour of u and update the current value in the map with this min key (weight)
        for (auto adj_vertex_itr = adj_list_weighted_edges[u].begin(); adj_vertex_itr != adj_list_weighted_edges[u].end(); ++adj_vertex_itr) {
            if (!( find(mstSet.begin(), mstSet.end(), adj_vertex_itr->first) != mstSet.end() ) && adj_vertex_itr->second < key.find(adj_vertex_itr->first)->second) {
                parent.find(adj_vertex_itr->first)->second = u;
                key.find(adj_vertex_itr->first)->second = adj_vertex_itr->second;
            }
        }
    }

    for (string vertex : mstSet) {
        min_spanning_tree.add_vertex(vertex); // add all the vertices to the tree from mstSet
    }

    bool is_first_iteration = true;
    for (auto itr = parent.begin(); itr != parent.end(); ++itr) {
        if (is_first_iteration) { // skip the first iteration since the A does not have parent to form an edge
            is_first_iteration = false;
            continue;
        }
        auto weight = key.find(itr->first)->second; // get the element in the map that match the adj vertex (itr->first)
        min_spanning_tree.add_edge(itr->second, itr->first, weight); // then add all the edges from parent with the weight
        // weight is guaranteed to be the weight of (itr->second, itr->first) because of this line: auto weight = key.find(itr->first)->second;
    }
    return min_spanning_tree;
}

/* helper function for test 11 */

template <typename T>
bool Graph<T>::has_cycle(string vertex, set<string>& visited, string parent) {
    // A {B, C, E}
    // B {A, E}
    // C {A, D, E}
    // D {C}
    // E {C, B, A}

    // A B E C A
    // B A C D E

    // DFS through each node and check if the next visit node is the parent
    // if it is not parent and has already visited then there is a cycle
    visited.insert(vertex);
    for (auto neighbour : get_neighbours(vertex)) {
        if (!(visited.find(neighbour) != visited.end())) { // if the neighbour has not been visited then recur
            if (has_cycle(neighbour, visited, vertex)) { // recur this neighbour then return true if there is a cycle
            return true;
            }
        }
        else if (neighbour != parent) { // if the neighbour has been visited and not equal to the parent then there is a cycle
            return true;
        }
    }
    return false;
}

/* helper function for test 12 */

template <typename T>
string Graph<T>::min_adj_vertex(map<string, T>& key, vector<string>& mstSet) {
    int min = INT_MAX;
    string min_vertex;
    // obtain the smallest vertex (that has the smallest value in the map key and has not been visited)
    // then return that vertex
    for (auto itr = adj_list_weighted_edges.begin(); itr != adj_list_weighted_edges.end(); ++itr) {
        if (!(find(mstSet.begin(), mstSet.end(), itr->first) != mstSet.end()) && key.find(itr->first)->second < min) {
            min = key.find(itr->first)->second;
            min_vertex = key.find(itr->first)->first;
        }
    }
    return min_vertex;
}