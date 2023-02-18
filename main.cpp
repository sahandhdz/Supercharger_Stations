#include "network.h"
#include <cmath>
#include <vector>
#include <algorithm>

using namespace std;
const double pi = 3.141592653589793;
const double speed = 105;

struct path_node{
	int idx;
	double g;
	double h;
};

double deg_to_rad (double deg){
	// Converts degree to radian
	return deg*pi/180;
}

double compute_distance(row &station_1, row &station_2){
	// Gets the latitude and longitude of two stattions and gives the distance between them in km
	double lat_1 = deg_to_rad(station_1.lat);
	double lon_1 = deg_to_rad(station_1.lon);
	double lat_2 = deg_to_rad(station_2.lat);
	double lon_2 = deg_to_rad(station_2.lon);
	double d = sin(lat_1)*sin(lat_2) + cos(lat_1)*cos(lat_2)*cos(lon_2-lon_1); // Haversine formula
	d = 6356.752 * acos(d);
	return d;
}


int find_idx(string station_name){
	// Gets the name of a station as string and gives its index (int) as the ouput
	for (int i=0; i<network.size(); i++){
		if (station_name == network[i].name){
			return i;
			break;
		}
	}
	return -1;
}

vector<int> find_reachable (int idx_start, double fuel){
	// Finds the set of all reachable nodes.
	// Input: The starter node/station & the current fuel/charge
	// Output: A vector including the index of all reachable nodes/stations
	vector<int> R {};
	for (int i=0; i<network.size(); i++){
		if ( (i != idx_start) && (compute_distance(network[idx_start], network[i]) <= fuel) ){
			R.push_back(i);
		}
	}
	return R;
}


double cost_g(int idx_1, int idx_2){
	// computes the cost of moving from station 1 (idx_1) to station 2 (idx_2), in time unit
	double d = compute_distance(network[idx_1], network[idx_2]);
	double g = d/speed + d/network[idx_2].rate; // distance/time + time to refuel/recharge
	return g;
}

double heuristic(int idx, int idx_goal){
	// Measures the distance between the given node and the final destination
	double d = compute_distance(network[idx], network[idx_goal]);
	return d;
}

bool compare(path_node s1, path_node s2){
	// To be used by the sort, to find the best station for the next step
	return (s1.g + s1.h) > (s2.g + s2.h);
}



vector <int> path_planning(int idx_start, int idx_goal){
	// A* Search
	// Input: The start node & the goal node
	// Output: The sequence of nodes/stations that constructs a path between the start node and the goal node
	double fuel = 320;
	vector<path_node> open_list {{idx_start, 0, 0}};
	vector<int> closed_idx {};
	vector<int> path_idx {};
	
	while (!open_list.empty()){
		sort(open_list.begin(), open_list.end(), compare);
		path_node current = open_list[open_list.size()-1];
		path_idx.push_back(current.idx);
		
		open_list.pop_back();
		if (current.idx == idx_goal){
			return path_idx;
		}
		
		vector <int> R_idx = find_reachable(current.idx, fuel);
		
		for (int i: R_idx){
			if (find(closed_idx.begin(), closed_idx.end(), i) == closed_idx.end()){
				double g = current.g + cost_g(current.idx, i);
				double h = heuristic(i, idx_goal);
				open_list.push_back({i, g, h});
				closed_idx.push_back(i);
			}
		}
		
	}
}

vector<double> path_distances(vector<int> p){
	// Input: the secuence of path nodes/stations
	// Output: the distance between each pair of subsequent nodes/stations in the path
	vector<double> d {};
	for (int i=1; i<p.size(); i++){
		d.push_back(compute_distance(network[p[i]], network[p[i-1]]));
	}
	return d;
}

vector<double> charging(vector<int> p, vector<double> d){
	// Computes the required charge/fuel at each station
	vector<double> charging_sequence = {0};
	double E = 320;
	for (int i=1; i<p.size()-2; i++){
		E -= d[i-1];
		if (network[p[i]].rate < network[p[i+1]].rate){
			if (E<d[i]){
				charging_sequence.push_back(d[i]-E);
				E = d[i];
			}
			else{
				charging_sequence.push_back(0);
			}
		}
		else{
			charging_sequence.push_back(320-E);
			E = 320;
		}
	}
	E -= d[d.size()-2];
	charging_sequence.push_back((d[d.size()-1]-E));
	charging_sequence.push_back(0);
	return charging_sequence;
}

void printout(vector<int> path, vector<double> charging_seq){
	// Computes the charging/fueling time at each station
	// Prints out the sequence of stations along with the fueling/charging times
	for (int i = 0; i<path.size(); i++){
		if (i == 0){
			std::cout << network[path[i]].name << ", ";
		}else if (i == path.size()-1){
			std::cout << network[path[i]].name;
		}else{
			std::cout << network[path[i]].name << ", " << charging_seq[i]/network[path[i]].rate + 0.01 << ", ";
		}
	}
	cout << "\n";
}



int main(int argc, char** argv)
{
	
    if (argc != 3)
    {
        std::cout << "Error: requires initial and final supercharger names" << std::endl;
        return -1;
    }
    
    std::string start_station = argv[1];
    std::string goal_station = argv[2];
	
	
	int idx_s = find_idx(start_station); // Index of the start node/station
	int idx_g = find_idx(goal_station);  // Index of the goal node/station
	
	
	vector<int> path = path_planning(idx_s, idx_g); // Computes a potentially shortest path
	vector<double> D = path_distances(path); // Computes the distance between susequent nodes/stations in the path
	vector<double> Charge_Sequence = charging(path, D); // Minimize the charging/fueling time at the intermediate stations
	
	printout(path, Charge_Sequence); // Prints the path and the chatging times, based on the required format 
	

    return 0;
}