#include "Graph.h"
#include <fstream>
#include <sstream>
#include <utility>
#include <iostream>
#include <queue>
#include <limits>
using namespace std;

Graph::Graph(const char* const & edgelist_csv_fn) {
    // TODO
    ifstream file(edgelist_csv_fn);      
    string line;                     
    while(getline(file, line)) {  
        istringstream ss(line);
        string u, v, w;
        if (getline(ss, u, ',') && getline(ss, v, ',') && getline(ss, w)) {
            double weight = stod(w);
            if (weight < 0) {
                cout << "Weights have to be positive for Djikstra";
                exit(1);
            }
            matrix[u].push_back(make_pair(v, weight));
            matrix[v].push_back(make_pair(u, weight));
        } else {
            exit(1);
            cout << "Please follow format for the nodes u, v, w";
        }
    }
    file.close();   
}

unsigned int Graph::num_nodes() {
    // TODO
    return matrix.size();
}

vector<string> Graph::nodes() {
    // TODO
    vector <string> temp;
    for (auto itr = matrix.begin(); itr != matrix.end(); itr++) {
        temp.push_back(itr->first);
    }
    return temp;
}

unsigned int Graph::num_edges() {
    // TODO
    unsigned int num_undir_edge = 0;
    for (auto itr = matrix.begin(); itr != matrix.end(); itr++) {
        num_undir_edge += itr->second.size();
    }
    return num_undir_edge/2;
}

unsigned int Graph::num_neighbors(string const & node_label) {
    // TODO
    auto itr = matrix.find(node_label);
    if (itr != matrix.end()) {
        return itr->second.size();
    }
    return 0;
}

double Graph::edge_weight(string const & u_label, string const & v_label) {
    // TODO
    auto itr = matrix.find(u_label);
    if (itr != matrix.end()) {
        const vector<pair<string, double>>& neighbors = itr->second;
        for (const pair<string, double>& neighbor : neighbors) {
            if (neighbor.first == v_label) {
                return neighbor.second;
            }
        }
    }
    return -1;
}

vector<string> Graph::neighbors(string const & node_label) {
    // TODO
    vector<string> nodes;
    auto itr = matrix.find(node_label);
    if (itr != matrix.end()) {
        for (const auto & edge : itr->second) {
            nodes.push_back(edge.first);
        }
    }
    return nodes;
}

vector<string> Graph::shortest_path_unweighted(string const & start_label, string const & end_label) {
    // TODO
    vector <string> path;
    map <string, string> parent;
    parent[start_label] = "";
    priority_queue <pair<double, string>, vector<pair<double, string>>, greater <pair<double,string>>> min_heap;

    min_heap.push(make_pair(0, start_label));
    if (start_label == end_label) {
        path.push_back(start_label);
        return path;
    }
    while(!min_heap.empty()) {
        string node = min_heap.top().second;
        double weight = min_heap.top().first;
        min_heap.pop();

        if (node == end_label) {
            while (node != "") {
                path.push_back(node);
                node = parent[node];
            }
            vector<string> temp;
            for (auto itr = path.rbegin(); itr != path.rend(); itr++) {
                temp.push_back(*itr);
            }
            return temp;
        }

        for (string neighbor : neighbors(node)) {
            if (parent.find(neighbor) == parent.end()) {
                parent[neighbor] = node; 
                min_heap.push(make_pair(weight + 1, neighbor));
            }
        }
    }
    return path;
}

vector<tuple<string,string,double>> Graph::shortest_path_weighted(string const & start_label, string const & end_label) {
    // TODO
    vector<tuple<string,string,double>> path;
    map <string, string> parent;
    map<string, double> weights;
    if (start_label == end_label) {
        path.emplace_back(make_tuple(start_label, start_label, -1));
        return path;
    }
    for (const auto& node : matrix) {
        weights[node.first] = numeric_limits<double>:: infinity();
        parent[node.first] = "";
    }
    weights[start_label] = 0;
    priority_queue <pair<double, string>, vector<pair<double, string>>, greater<pair<double,string>>> min_heap;
    min_heap.push(make_pair(0, start_label));

    while(!min_heap.empty()) {
        string node = min_heap.top().second;
        double weight = min_heap.top().first;
        min_heap.pop();

        if (weight > weights[node]) {
            continue;
        }

        if (node == end_label) {
            break;
        }
        for (const auto& neighbor : matrix[node]) {
            string node_neigh = neighbor.first;
            double weight_neigh = edge_weight(node, node_neigh);
            double new_dist = weight_neigh + weights[node];

            if (weight_neigh != -1 && new_dist < weights[node_neigh]) {
                weights[node_neigh] = new_dist;
                parent[node_neigh] = node;
                min_heap.push(make_pair(new_dist, node_neigh));
            }
        }
    }

    if(parent[end_label] == "") {
        return path;
    }
    string node = end_label;
    while (node != start_label) {
        string prev_node = parent[node];
        double edge = edge_weight(prev_node, node);
        path.emplace_back(prev_node, node, edge);
        node = prev_node;
    }
    vector<tuple<string, string, double>> reversed_path(path.rbegin(), path.rend());
    return reversed_path;
}

vector<vector<string>> Graph::connected_components(double const & threshold) {
    // TODO
    vector<vector<string>> temp;
    map <string, bool> checkVisit;
    for (const auto& pair : matrix) {
        const string & node = pair.first;
        if (!checkVisit[node]) {
            vector<string> comp;
            queue <string> que;
            que.push(node);
            checkVisit[node] = true;

            while (!que.empty()) {
                string cur = que.front();
                que.pop();
                comp.push_back(cur);

                for (const auto& neighbor : matrix[cur]) {
                    const string& node_neigh = neighbor.first;
                    double weight_neigh = neighbor.second;

                    if (!checkVisit[node_neigh] && weight_neigh <= threshold) {
                        que.push(node_neigh);
                        checkVisit[node_neigh] = true;
                    }
                }
            }
            temp.push_back(comp);
        }
    }   
    return temp;
}

double Graph::smallest_connecting_threshold(string const & start_label, string const & end_label) {
    // TODO
    if (start_label == end_label) {
        return 0;
    }
    if (!matrix.count(start_label) || !matrix.count(end_label)) {
        return -1;
    }
    map <string, double> smallThres;
    for (const auto& node : matrix) {
        smallThres[node.first] = numeric_limits<double>:: max();
    }
    priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> min_heap;
    min_heap.push(make_pair(0, start_label));
    while(!min_heap.empty()) {
        double weight = min_heap.top().first;
        string node = min_heap.top().second;
        min_heap.pop();

        if (smallThres[node] < weight) {
            continue; 
        }

        if (node == end_label) {
            return weight;
        }

        for (const auto& neighbor : matrix[node]) {
            string node_neigh = neighbor.first;
            double weight_neigh = neighbor.second;
            double total = max(weight, weight_neigh);

            if (total < smallThres[node_neigh]) {
                smallThres[node_neigh] = total;
                min_heap.push(make_pair(total, node_neigh));
            }
        }
    }
    return -1;
}
