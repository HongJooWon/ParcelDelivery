#include "inet/common/INETMath.h"
//-----------------------------------
#define INET_EXPORTS
//-----------------------------------
#include "DroneNetMob.h"
//#included "DroneDeliveryAlgorithms.h"
#include <bitset>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unordered_map>
#include <queue>
#include <ctime>
#include <string>

using namespace std;
namespace inet {

Define_Module(DroneNetMob);

Coord originPos = Coord(0, 0, 0);

//-------------------------------------------------- Utility Functions --------------------------------------------------

bool sortByScore(const ParcelScore& a, const ParcelScore& b) {
    return a.score > b.score;
}

double dist(Coord a, Coord b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

//정렬된 리스트 경로와 무게 변화에 따른 battery consumtion
double batteryCalculation(Coord a, Coord b, double carriedWeight, double speed) {
    double distance = dist(a, b);
    double batteryConsumption = 0;

    //택배없이 일반적인 horizontal moving일 때, E = 308.709t - 0.852
    //택배가 있을 때, L in grams, E = (0.311L + 1.735)*t
    double time = distance / speed;
    if (carriedWeight == 0) {
        batteryConsumption = 308.709 * time - 0.852;
    } else {
        batteryConsumption = (0.311 * carriedWeight + 1.735) * time;
    }

    return batteryConsumption;
}

//Add all the destinations in the series of parcels vector
void addallDist(vector<parcel>& parcels) {
    double totalDist = 0;
    //add origin to first parcel
    totalDist += dist(originPos, parcels[0].parceldest);

    for(int i=0; i<parcels.size()-1; i++) {
        totalDist += dist(parcels[i].parceldest, parcels[i+1].parceldest);
    }
    
    totalDist += dist(parcels[parcels.size()-1].parceldest, originPos);
    cout << "add all distance: " << totalDist << endl;
}

// Minimum Spanning Tree
double calculateMST(const vector<vector<double>>& distances, const vector<bool>& included) {
    int n = distances.size();
    double mst_cost = 0;
    vector<bool> in_mst = included;
    vector<double> min_edge(n, numeric_limits<double>::max());

    // 시작 노드 선택 (포함되지 않은 첫 번째 노드)
    int start = 0;
    for (int i = 0; i < n; ++i) {
        if (!included[i]) {
            start = i;
            break;
        }
    }
    min_edge[start] = 0;

    for (int i = 0; i < n; ++i) {
        if (included[i]) continue;  // 이미 포함된 노드는 건너뜀

        int u = -1;
        double min = numeric_limits<double>::max();
        for (int v = 0; v < n; ++v) {
            if (!in_mst[v] && min_edge[v] < min) {
                min = min_edge[v];
                u = v;
            }
        }

        if (u == -1) break;  // 더 이상 연결할 노드가 없음

        in_mst[u] = true;
        mst_cost += min_edge[u];

        for (int v = 0; v < n; ++v) {
            if (!in_mst[v] && distances[u][v] < min_edge[v]) {
                min_edge[v] = distances[u][v];
            }
        }
    }

    return mst_cost;
}

//중복 좌표 제거
void remove_dupcoordinates(vector<parcel>& parcels, int& carriedParcels, int& totalparcel) {
    int dup = 0;
    unordered_map<string, int> coord_map;

    cout << "carriedParcels with Duplicates: " << parcels.size() << endl;

    for (int i = 0; i < parcels.size(); i++) {
        string coord = to_string(parcels[i].parceldest.x) + "," + to_string(parcels[i].parceldest.y);

        if (coord_map.find(coord) == coord_map.end()) {
            // 좌표가 처음 발견된 경우, 해당 좌표와 인덱스를 맵에 추가
            coord_map[coord] = i;
        } else {
            // 좌표가 이미 존재하는 경우, 해당 택배 아이템을 제거
            dup++;
            parcels.erase(parcels.begin() + i);
            i--;  // 인덱스를 조정하여 제거된 요소를 건너뛰지 않도록 함
            carriedParcels--; //중복된 만큼 전체 택배 수를 감소
            totalparcel--;
        }
    }

    cout << "carriedParcels without Duplicates: " << parcels.size() << endl;
}

void removeDuplicatesInDepot(vector<parcel>* parcel_depot) {
    if (parcel_depot->empty()) return;

    unordered_map<string, int> coord_map;
    vector<parcel> unique_parcels;
    int dup = 0;

    cout << "Initial depot size with duplicates: " << parcel_depot->size() << endl;

    for (const auto& p : *parcel_depot) {
        string coord = to_string(p.parceldest.x) + "," + to_string(p.parceldest.y);

        if (coord_map.find(coord) == coord_map.end()) {
            // 좌표가 처음 발견된 경우
            coord_map[coord] = unique_parcels.size();
            unique_parcels.push_back(p);
        } else {
            // 좌표가 이미 존재하는 경우
            dup++;
            // 여기서 중복된 택배를 처리하는 추가 로직을 넣을 수 있음
            // 예: 중복된 택배들 중 가장 무거운 것을 선택하거나
            // 우선순위가 높은 것을 선택하는 등
        }
    }

    // 중복이 제거된 택배 리스트로 업데이트
    *parcel_depot = unique_parcels;

    cout << "Duplicates removed: " << dup << endl;
    cout << "Final depot size without duplicates: " << parcel_depot->size() << endl;
}

//-------------------------------------------------- Greedy Choice --------------------------------------------------

//distance-greedy
vector<parcel> NearestFirst(vector<parcel>& parcels, double& distance, double speed, double& energy) {
    // 1. 좌표별로 그룹화
    unordered_map<string, GroupedParcel> parcelGroups;
    for (const auto& p : parcels) {
        string coord = to_string(p.parceldest.x) + "," + to_string(p.parceldest.y);
        parcelGroups[coord].parcels.push_back(p);
        parcelGroups[coord].totalWeight += p.weight;
        parcelGroups[coord].dest = p.parceldest;
    }

    vector<GroupedParcel> destinations;
    for (const auto& group : parcelGroups) {
        destinations.push_back(group.second);
    }

    // 2. 거리 기반 경로 계산
    vector<parcel> sortedParcels;
    vector<bool> visited(destinations.size(), false);
    Coord currPos = originPos;
    double totalDist = 0;

    cout << "Nearest First calculating route for " << destinations.size() << " destinations\n";

    // 가장 가까운 목적지 순으로 방문
    while (sortedParcels.size() < parcels.size()) {
        double minDist = numeric_limits<double>::max();
        int nextDest = -1;

        // 가장 가까운 미방문 목적지 찾기
        for (int i = 0; i < destinations.size(); i++) {
            if (!visited[i]) {
                double d = dist(currPos, destinations[i].dest);
                if (d < minDist) {
                    minDist = d;
                    nextDest = i;
                }
            }
        }

        if (nextDest != -1) {
            // 해당 목적지의 모든 택배를 순서대로 추가
            const auto& selectedGroup = destinations[nextDest];
            
            cout << "Selected destination (" << selectedGroup.dest.x << "," 
                 << selectedGroup.dest.y << ") with " << selectedGroup.parcels.size() 
                 << " parcels, total weight: " << selectedGroup.totalWeight << endl;

            // 해당 목적지의 모든 택배 추가
            for (const auto& p : selectedGroup.parcels) {
                sortedParcels.push_back(p);
            }

            // 거리와 에너지 계산 (목적지당 한 번만)
            totalDist += minDist;
            energy += batteryCalculation(currPos, selectedGroup.dest, selectedGroup.totalWeight, speed);
            
            currPos = selectedGroup.dest;
            visited[nextDest] = true;
        }
    }

    // 출발지로 돌아가기
    totalDist += dist(currPos, originPos);
    energy += batteryCalculation(currPos, originPos, 0, speed);
    distance += totalDist;

    // 결과 출력
    cout << "Nearest-First Algorithm Results:" << endl;
    cout << "Total destinations visited: " << destinations.size() << endl;
    cout << "Total parcels delivered: " << sortedParcels.size() << endl;
    cout << "Total distance: " << distance << endl;
    cout << "Total energy consumption: " << energy << " mAh" << endl;
    
    // 경로 세부 정보 출력
    cout << "\nDelivery Route Details:" << endl;
    currPos = originPos;
    unordered_map<string, int> deliveryCount;
    for (const auto& p : sortedParcels) {
        string coord = to_string(p.parceldest.x) + "," + to_string(p.parceldest.y);
        deliveryCount[coord]++;
    }
    
    for (const auto& count : deliveryCount) {
        cout << "Destination " << count.first << ": " 
             << count.second << " parcels delivered" << endl;
    }

    return sortedParcels;
}

//weight-greedy
vector<parcel> HeaviestFirst(vector<parcel>& parcels, double& distance, double speed, double& energy){
    vector<parcel> sortedParcels = {};
    vector<bool> visited(parcels.size(), false);
    Coord currPos = originPos;
    double totalDist = 0;
    while(sortedParcels.size() < parcels.size()) {
        double minWeight = -1;
        int nextParcel = -1;
        for(int i=0; i<parcels.size(); i++) {
            if(!visited[i]) {
                if(parcels[i].weight < minWeight || minWeight == -1) {
                    minWeight = parcels[i].weight;
                    nextParcel = i;
                }
            }
        }
        //cout << "Current Position: " << currPos.x << ", " << currPos.y << " Next position: " << parcels[nextParcel].parceldest.x << ", " << parcels[nextParcel].parceldest.y << " Distance: " << dist(currPos, parcels[nextParcel].parceldest) << endl;
        totalDist += dist(currPos, parcels[nextParcel].parceldest);
        energy += batteryCalculation(currPos, parcels[nextParcel].parceldest, parcels[nextParcel].weight, speed);
        //cout  << "minWeight: " << minWeight << " Total distance: " << totalDist << endl;

        visited[nextParcel] = true;
        sortedParcels.push_back(parcels[nextParcel]);
        currPos = parcels[nextParcel].parceldest;
    }

    totalDist += dist(currPos, originPos); //출발지로 돌아가는 거리
    energy += batteryCalculation(currPos, originPos, 0, speed); //출발지로 돌아가는 배터리 소모까지 계산
    distance += totalDist;
    cout << "Heaviest-Greedy TSP distance: " << distance << endl;
    cout << "Heaviest-Greedy TSP energy: " << energy << endl;

    return sortedParcels;
}

//balanced-greedy
// Efficient Drone Trajectory Design algorithm implementation
// std::vector<parcel> efficientTrajectoryDesign(std::vector<parcel>& parcels, double& totalDistance);
// MissionParcels, tspDistance, droneremainingbattery, getMaxSpeed()
vector<parcel> efficientTrajectoryDesign(vector<parcel>& parcels, double& totalDistance, double speed, double& energy) {
    vector<parcel> sortedParcels;
    if (parcels.empty()) return sortedParcels;

    // 1. 전체 무게(W)와 거리(D) 계산
    double totalWeight = 0;
    double totalPathDistance = 0;
    Coord currentPos = originPos;
    
    for (const auto& p : parcels) {
        totalWeight += p.weight;
        totalPathDistance += dist(currentPos, p.parceldest);
        currentPos = p.parceldest;
    }
    totalPathDistance += dist(currentPos, originPos);

    // 2. 각 택배의 Vi 값 계산 및 전체 V 계산
    vector<ParcelScore> parcelScores;
    double totalV = 0;  // 전체 V 값
    
    for (const auto& p : parcels) {
        ParcelScore ps;
        ps.p = p;
        
        double w_i = p.weight / totalWeight;
        double d_i = dist(originPos, p.parceldest) / totalPathDistance;
        
        const double ALPHA = 0.4; //무게 가중치
        const double BETA = 0.6; //거리 가중치
        ps.score = (ALPHA * w_i + BETA * d_i);  // Vi 계산
        totalV += ps.score;  // 전체 V 계산
        parcelScores.push_back(ps);
    }

    // 3. V'i = Vi/V 계산
    for (auto& ps : parcelScores) {
        ps.score = ps.score / totalV;  // V'i 계산
    }

    // 4. V'i 값에 따라 정렬 (내림차순)
    sort(parcelScores.begin(), parcelScores.end(), sortByScore);

    // 5. 정렬된 순서대로 경로 생성
    for (const auto& ps : parcelScores) {
        sortedParcels.push_back(ps.p);
        cout << "Selected parcel with V'i = " << ps.score << ", weight = " 
             << ps.p.weight << ", position = (" << ps.p.parceldest.x 
             << "," << ps.p.parceldest.y << ")" << endl;
    }

    // 6. 총 거리 계산
    currentPos = originPos;
    for (const auto& p : sortedParcels) {
        totalDistance += dist(currentPos, p.parceldest);
        energy += batteryCalculation(currentPos, p.parceldest, p.weight, speed);
        currentPos = p.parceldest;
    }
    totalDistance += dist(currentPos, originPos);
    energy += batteryCalculation(currentPos, originPos, 0, speed);

    cout << "Balanced Greedy TSP Distance: " << totalDistance << endl;
    cout << "Balanced Greedy TSP Energy: " << energy << endl;
    return sortedParcels;
}

//greedy algorithm for TSP with battery consumption
vector<parcel> greedyTSP_B(vector<parcel>& parcels, double& distance, double carriedWeight, double speed){
    vector<parcel> sortedParcels = {};
    vector<bool> visited(parcels.size(), false);
    Coord currPos = originPos;

    //battery consumption을 거리와 현재 적재중인 택배의 무게를 고려하여 계산할 것
    double totalConsumption = 0;
    double totalDist = 0;

    //현재 좌표에서 Energy Consumption이 가장 큰 택배를 선택
    while(sortedParcels.size() < parcels.size()) {
        double minEnergy = -1;
        int nextParcel = -1;

        for(int i=0; i<parcels.size(); i++) {
            if(!visited[i]) {
                double currEnergy = batteryCalculation(currPos, parcels[i].parceldest, carriedWeight, speed);
                if(currEnergy < minEnergy || minEnergy == -1) {
                    minEnergy = currEnergy;
                    nextParcel = i;
                }
            }
        }

        totalConsumption += minEnergy;
        totalDist += dist(currPos, parcels[nextParcel].parceldest);

        visited[nextParcel] = true;
        sortedParcels.push_back(parcels[nextParcel]);
        currPos = parcels[nextParcel].parceldest;
    }

    totalConsumption += batteryCalculation(currPos, originPos, carriedWeight, speed); //출발지로 돌아가는 배터리 소모까지 계산
    totalDist += dist(currPos, originPos); //출발지로 돌아가는 거리까지 계산

    distance += totalDist;
    cout << "Energy Greedy distance: " << distance << endl;

    cout << "Total energy consumption: " << totalConsumption << endl;

    return sortedParcels;
}

//-------------------------------------------------- Branch and Bound --------------------------------------------------

// 1-tree lower bound를 계산하는 함수
double calculate1TreeLowerBound(const vector<vector<double>>& travel, int start, const vector<bool>& visited) {
    int n = travel.size();
    vector<bool> mst_included = visited;
    mst_included[start] = true;  // 시작 노드 제외

    double mst_cost = calculateMST(travel, mst_included);

    // 1-tree를 위한 추가 엣지 계산
    vector<double> min_edges;
    for (int i = 0; i < n; ++i) {
        if (i != start && !visited[i]) {
            min_edges.push_back(travel[start][i]);
        }
    }
    sort(min_edges.begin(), min_edges.end());

    double one_tree_cost = mst_cost;
    if (min_edges.size() >= 2) {
        one_tree_cost += min_edges[0] + min_edges[1];
    }

    return one_tree_cost;
}

void dfs(int start, int next, double value, vector<int>& visited, int n, vector<vector<double>>& travel, double& min_value, vector<int>& path) {
    // 현재까지 방문한 노드 표시
    vector<bool> is_visited(n, false);
    for (int v : visited) is_visited[v] = true;

    // 1-tree lower bound 계산
    double lower_bound = value + calculate1TreeLowerBound(travel, next, is_visited);
    
    // 가지치기: 현재 경로의 lower bound가 지금까지의 최소값보다 크거나 같으면 탐색 중단
    if (lower_bound > min_value) {
        return;
    }

    // 모든 노드를 방문한 경우
    if (visited.size() == n) {
        // 시작점으로 돌아갈 수 있는지 확인
        if (travel[next][start] != 0) {
            double total_distance = value + travel[next][start];
            // 더 짧은 경로를 찾은 경우 최소값과 최적 경로 갱신
            if (total_distance < min_value) {
                min_value = total_distance;
                path = visited;
            }
        }
        return;
    }

    // 다음 방문할 노드 탐색
    for (int i = 0; i < n; i++) {
        if (travel[next][i] != 0 && i != start && find(visited.begin(), visited.end(), i) == visited.end()) {
            visited.push_back(i);
            // 재귀적으로 DFS 수행
            dfs(start, i, value + travel[next][i], visited, n, travel, min_value, path);
            // 백트래킹
            visited.pop_back();
        }
    }
}

// BnB Tsp
vector<parcel> dfs_bnb(vector<parcel>& parcels, double& distance, int& carriedParcels) {

    //numparcel 할당 문제---
    cout << "carriedParcels: " << parcels.size() << endl; 

    // 시작점을 (0,0)으로 설정하고 택배 리스트에 추가
    parcel startParcel;
    startParcel.parcelID = -1;
    startParcel.parceldest = originPos;
    parcels.insert(parcels.begin(), startParcel);
    
    // 택배 리스트를 출력
    
    for(int i=0; i<parcels.size(); i++) {
        cout << "Parcel to solve" << parcels[i].parcelID << " : " << parcels[i].parceldest.x << " " << parcels[i].parceldest.y << endl;
    }

    //remove_dupcoordinates(parcels, carriedParcels);

    cout << "After removing duplicates" << endl;

    // 택배 리스트를 출력
    for(int i=0; i<parcels.size(); i++) {
        cout << "Parcel to solve" << parcels[i].parcelID << " : " << parcels[i].parceldest.x << " " << parcels[i].parceldest.y << endl;
    }

    // 택배의 수를 계산
    int n = parcels.size();

    // 각 택배 간의 거리를 계산하여 행렬에 저장
    vector<vector<double>> travel(n, vector<double>(n, 0));
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            travel[i][j] = dist(parcels[i].parceldest, parcels[j].parceldest);
        }
    }

    // 최단 거리를 무한대로 초기화
    double min_value = numeric_limits<double>::infinity();
    // 최적 경로를 저장할 변수를 초기화
    vector<int> path;
    // 방문 리스트를 초기화하고, 시작점을 추가
    vector<int> visited = {0};
    double ans = 0;

    // DFS를 시작
    dfs(0, 0, 0, visited, n, travel, min_value, path);

    // 최적 경로에 시작점으로 돌아오는 것 추가
    path.push_back(0);

    // 최적 경로에 따라 택배를 정렬 (시작점 제외)
    vector<parcel> sorted_parcels;
    for(int i = 1; i < path.size() - 1; i++) {
        sorted_parcels.push_back(parcels[path[i]]);
    }
    
    //print the sorted parcels
    cout << "BnB Sorted parcels: ";
    for(int i=0; i<sorted_parcels.size(); i++) {
        cout << sorted_parcels[i].parcelID << " ";
    }
    cout << endl;

    //현재 들고있는 리스트에 대한 최소거리
    cout << "Minimum distance: " << min_value << endl;

    //현재까지 축적된 거리
    distance += min_value;
    cout << "BnB total distance increased: " << distance << endl;

    // 정렬된 택배 리스트를 반환
    return sorted_parcels;
}

//Battery consumption based DFS
void dfs_b(int start, int next, double value, vector<int>& visited, int n, vector<vector<double>>& travel, double& min_value, vector<int>& path) {
    // 현재까지 방문한 노드 표시
    vector<bool> is_visited(n, false);
    for (int v : visited) is_visited[v] = true;

    // 1-tree lower bound 계산
    double lower_bound = value + calculate1TreeLowerBound(travel, next, is_visited);
    
    // 가지치기: 현재 경로의 lower bound가 지금까지의 최소값보다 크거나 같으면 탐색 중단
    if (lower_bound > min_value) {
        return;
    }

    // 모든 노드를 방문한 경우
    if (visited.size() == n) {
        // 시작점으로 돌아갈 수 있는지 확인
        if (travel[next][start] != 0) {
            double total_distance = value + travel[next][start];
            // 더 짧은 경로를 찾은 경우 최소값과 최적 경로 갱신
            if (total_distance < min_value) {
                min_value = total_distance;
                path = visited;
            }
        }
        return;
    }

    // 다음 방문할 노드 탐색
    for (int i = 0; i < n; i++) {
        if (travel[next][i] != 0 && i != start && find(visited.begin(), visited.end(), i) == visited.end()) {
            visited.push_back(i);
            // 재귀적으로 DFS 수행
            dfs(start, i, value + travel[next][i], visited, n, travel, min_value, path);
            // 백트래킹
            visited.pop_back();
        }
    }
}

// BnB Tsp
vector<parcel> bnb_B(vector<parcel>& parcels) {

    //numparcel 할당 문제---
    cout << "carriedParcels: " << parcels.size() << endl; 

    // 시작점을 (0,0)으로 설정하고 택배 리스트에 추가
    parcel startParcel;
    startParcel.parcelID = -1;
    startParcel.parceldest = originPos;
    parcels.insert(parcels.begin(), startParcel);
    
    // 택배 리스트를 출력
    
    for(int i=0; i<parcels.size(); i++) {
        cout << "Parcel to solve" << parcels[i].parcelID << " : " << parcels[i].parceldest.x << " " << parcels[i].parceldest.y << endl;
    }

    cout << "After removing duplicates" << endl;

    // 택배 리스트를 출력
    for(int i=0; i<parcels.size(); i++) {
        cout << "Parcel to solve" << parcels[i].parcelID << " : " << parcels[i].parceldest.x << " " << parcels[i].parceldest.y << endl;
    }

    // 택배의 수를 계산
    int n = parcels.size();

    // 각 택배 간의 에너지 소모량을 계사하여 행렬에 저장
    vector<vector<double>> travel(n, vector<double>(n, 0));
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            travel[i][j] = batteryCalculation(parcels[i].parceldest, parcels[j].parceldest, parcels[j].weight, 1);
        }
    }

    // 최소 소모량을 무한대로 초기화
    double min_value = numeric_limits<double>::infinity();
    // 최적 경로를 저장할 변수를 초기화
    vector<int> path;
    // 방문 리스트를 초기화하고, 시작점을 추가
    vector<int> visited = {0};
    double ans = 0;

    // DFS를 시작
    dfs(0, 0, 0, visited, n, travel, min_value, path);

    // 최적 경로에 시작점으로 돌아오는 것 추가
    path.push_back(0);

    // 최적 경로에 따라 택배를 정렬 (시작점 제외)
    vector<parcel> sorted_parcels;
    for(int i = 1; i < path.size() - 1; i++) {
        sorted_parcels.push_back(parcels[path[i]]);
    }
    
    //print the sorted parcels
    cout << "Energy BnB Sorted parcels: ";
    for(int i=0; i<sorted_parcels.size(); i++) {
        cout << sorted_parcels[i].parcelID << " ";
    }
    cout << endl;

    //현재 들고있는 리스트에 대한 최소거리
    cout << "Minimum Energy Consumption: " << min_value << endl;

    // 정렬된 택배 리스트를 반환
    return sorted_parcels;
}

//-------------------------------------------------- Dynamic Programming --------------------------------------------------
//optimal path가 수행되는지 확인
double dynamic(vector<parcel>& parcels, int pos, int visited, vector<vector<double>>& dp, vector<vector<double>>& distance, double currDist, double& ans, vector<int>& optimalPath, vector<int>& path) {
    path.push_back(pos); // add the current position to the path
    if(visited == ((1<<parcels.size()) - 1)) {
        ans = min(ans, currDist + distance[pos][0]); // update the minimum distance
        if(currDist + distance[pos][0] == ans) {
            optimalPath = path; // update the optimal path
        }
    }
    if(dp[pos][visited] != -1 && currDist >= dp[pos][visited]) {
        path.pop_back(); // remove the current position from the path
        return ans; // prune the branch if the current distance is already greater than the previously calculated distance
    }

    for(int i=0; i<parcels.size(); i++) {
        if((visited & (1<<i)) == 0) { // if not visited
            double newDist = currDist + distance[pos][i];
            if(newDist < ans) { // only explore the branch if the new distance is less than the current minimum distance
                dynamic(parcels, i, visited | (1<<i), dp, distance, newDist, ans, optimalPath, path);
            }
        }
    }
    dp[pos][visited] = currDist; // update the previously calculated distance
    path.pop_back(); // remove the current position from the path
    return ans;
}

vector<parcel> dp_tsp(vector<parcel>& parcels, double& totaldistance, int& carriedParcels, int& totalparcel) {
    cout << "Solving DP" << endl;
    //test
    remove_dupcoordinates(parcels, carriedParcels, totalparcel);

    // 시작점(원점) 추가
    parcel startParcel;
    startParcel.parcelID = -1;
    startParcel.parceldest = originPos;
    parcels.insert(parcels.begin(), startParcel);

    int n = parcels.size();
    vector<vector<double>> distance(n, vector<double>(n));
    for(int i=0; i<n; i++) {
        for(int j=0; j<n; j++) {
            distance[i][j] = dist(parcels[i].parceldest, parcels[j].parceldest);
        }
    }

    vector<vector<double>> dp(n, vector<double>(1<<n, -1));
    double ans = 1e9;
    vector<int> optimalPath;
    vector<int> path;
    dynamic(parcels, 0, 1, dp, distance, 0, ans, optimalPath, path);

    // 최적 경로에 시작점으로 돌아오는 것 추가
    optimalPath.push_back(0);

    // 정렬된 택배 리스트 생성 (시작점 제외)
    vector<parcel> sortedParcels;
    for(int i=1; i<optimalPath.size()-1; i++) {
        sortedParcels.push_back(parcels[optimalPath[i]]);
    }

    //마지막에 꼭 0이 추가된다
    //print the sorted parcels
    cout << "DP Sorted parcels: ";
    for(int i=0; i<sortedParcels.size(); i++) {
        cout << sortedParcels[i].parcelID << " ";
    }
    
    cout << endl;

    cout << "DP minimum distance: " << ans << endl;
    //add the distance to the total distance
    totaldistance += ans;

    return sortedParcels;
}

DroneNetMob::DroneNetMob()
{
    flag_original = false;
    gen = 0;
    flagArrangedDepot = false;
    OngoingMission = false;
    isEnd = false;
}

void DroneNetMob::initialize(int stage)
{
    LineSegmentsMobilityBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        rad heading = deg(par("initialMovementHeading"));
        rad elevation = deg(par("initialMovementElevation"));
        changeIntervalParameter = &par("changeInterval");
        angleDeltaParameter = &par("angleDelta");
        rotationAxisAngleParameter = &par("rotationAxisAngle");
        speedParameter = &par("speed");
        quaternion = Quaternion(EulerAngles(heading, -elevation, rad(0)));
        WATCH(quaternion);

        //initialize the drone
        numdst = &par("ndst");
        ox =&par("initialX");
        oy = &par("initialY");
        oz = &par("initialZ");
        originPos.x = ox->doubleValue();
        originPos.y = oy->doubleValue();
        originPos.z = oz->doubleValue();
        droneWeightCapacity =  (&par("weightCapacity"))->doubleValue();
        droneremainingbattery =  (&par("remainingBattery"))->doubleValue();
        carriedWeight = 0;
        selectionMethod = (&par("parcelSelectionMethod"))->intValue();

        //When it is the first drone
        if (strcmp(getParentModule()->getFullName(), "drone[0]") == 0){
            cout << " Name -----> " << getParentModule()->getFullName() <<endl;
            //print selectionmethod
            cout << " Selection Method -----> " << selectionMethod <<endl;
            if (numdst->intValue() > 0){
                destGen (numdst->intValue());
            }
            //택배 생성
            int np = (&par("npar"))->intValue();
            parcelsDefinition(np);

            // make a copy of the parcel list for the same parcel list for all drones
            shared_parcel_depot = parcel_depot;
        } else {
            // other drones copy the parcel list from the first drone
            cModule* drone0 = getParentModule()->getParentModule()->getSubmodule("drone", 0);
            DroneNetMob* drone0Mob = check_and_cast<DroneNetMob*>(drone0->getSubmodule("mobility"));
            
            dst = drone0Mob->dst;  // 목적지 리스트 복사
            
            
            // 각 드론이 독립적인 택배 리스트를 가지도록 함
            parcel_depot.clear();
            for(const auto& p : drone0Mob->shared_parcel_depot) {
                parcel_depot.push_back(p);
            }
        }

        cout << "Drone " << getParentModule()->getFullName() << " using algorithm: " << selectionMethod << endl;
    }
}

void DroneNetMob::orient()
{
    if (faceForward)
        lastOrientation = quaternion;
}

void DroneNetMob::setTargetPosition()
{
    if (flag_original){
        rad angleDelta = deg(angleDeltaParameter->doubleValue());
        rad rotationAxisAngle = deg(rotationAxisAngleParameter->doubleValue());
        Quaternion dQ = Quaternion(Coord::X_AXIS, rotationAxisAngle.get()) * Quaternion(Coord::Z_AXIS, angleDelta.get());
        quaternion = quaternion * dQ;
        quaternion.normalize();
        Coord direction = quaternion.rotate(Coord::X_AXIS);

        simtime_t nextChangeInterval = *changeIntervalParameter;
        EV_DEBUG << "interval: " << nextChangeInterval << endl;
        sourcePosition = lastPosition;
        targetPosition = lastPosition + direction * (*speedParameter) * nextChangeInterval.dbl();
        previousChange = simTime();
        nextChange = previousChange + nextChangeInterval;
    }
    else{
        EV_INFO << "target setting " << endl;
        simtime_t nextChangeInterval = *changeIntervalParameter;
        sourcePosition = lastPosition;
        //print the source position
        EV_INFO << "source position: " << sourcePosition << endl;
//            destination = missionPathNextDest(originPos);
        targetPosition = missionPathNextDest(lastPosition);
        previousChange = simTime();
        nextChange = previousChange + nextChangeInterval;
    }
}

void DroneNetMob::move()
{
    if (flag_original){
        simtime_t now = simTime();
        rad dummyAngle;
        if (now == nextChange) {
            lastPosition = targetPosition;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
            EV_INFO << "reached current target position = " << lastPosition << endl;
            setTargetPosition();
            cout << "flag target position = " << targetPosition << ", next change = " << nextChange << endl;
            lastVelocity = (targetPosition - lastPosition) / (nextChange - simTime()).dbl();
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
        }
        else if (now > lastUpdate) {
            ASSERT(nextChange == -1 || now < nextChange);
            double alpha = (now - previousChange) / (nextChange - previousChange);
            lastPosition = sourcePosition * (1 - alpha) + targetPosition * alpha;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
        }
    }
    else{
        simtime_t now = simTime();
        rad dummyAngle;
        if (now == nextChange) {
            lastPosition = targetPosition;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
            EV_INFO << "reached current target position = " << lastPosition << endl;
            setTargetPosition();
            //cout << "new target position = " << targetPosition << ", next change = " << nextChange << endl;

            lastVelocity = (targetPosition - lastPosition) / (nextChange - simTime()).dbl();
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
            //다음 위치까지 배터리 소모량 계산
            //처음 위치는 계산안함
            if(lastPosition != originPos){
                batteryConsumption += batteryCalculation(lastPosition, targetPosition, carriedWeight, getMaxSpeed());

                //cout << getParentModule()->getFullName() << "- Battery consumed: " << batteryConsumption << " mAh" << endl;

            }

        }
        else if (now > lastUpdate) {
            ASSERT(nextChange == -1 || now < nextChange);
            double alpha = (now - previousChange) / (nextChange - previousChange);
            lastPosition = sourcePosition * (1 - alpha) + targetPosition * alpha;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
            // if (strcmp(getParentModule()->getFullName(), "drone[1]") == 0){
            //     cout <<"Vel = " << lastVelocity.x <<"  ; " << lastVelocity.x << "  ; " << lastVelocity.z << endl;
            // }

        }
    }
}

double DroneNetMob::getMaxSpeed() const
{
    return speedParameter->isExpression() ? NaN : speedParameter->doubleValue();
}

void DroneNetMob::destGen(int ndst){
    //srand(time(0)); // 현재 시간을 시드로 사용하여 난수 생성기 초기화
    for (unsigned int i = 0; i < numdst->intValue(); i++){
        Coord nextdst;
        nextdst.x = rand() % 600;
        nextdst.y = rand() % 400;
        nextdst.z = 0;
        dst.push_back(nextdst);

        cout << "Destination: " << nextdst.x << ", " << nextdst.y << endl;
    }

}

//택배 생성
// void DroneNetMob::parcelsDefinition (string& filename){
//     ifstream parcelFile;
//     parcelFile.open(filename);

//     if (!parcelFile.is_open()){
//         cout << "File not found" << endl;
//     }

//     string line;
//     int nparcels = 0;
//     while (getline(parcelFile, line)){
//         parcel tmpparcel;
//         parcel *p;

//         // each line = id weight x y 한줄씩 읽어서 택배 생성
//         stringstream ss(line);
//         string id, weight, x, y;
//         getline(ss, id, ' ');
//         getline(ss, weight, ' ');
//         getline(ss, x, ' ');
//         getline(ss, y, ' ');

//         tmpparcel.parcelID = stoi(id);
//         tmpparcel.weight = stoi(weight);
//         tmpparcel.parceldest.x = stoi(x);
//         tmpparcel.parceldest.y = stoi(y);

//         parcel_depot.push_back(tmpparcel);

//         nparcels++;
//     }
// }


void DroneNetMob::parcelsDefinition (int nparcels){
    for (unsigned int i = 0; i < nparcels; i++){
        parcel tmpparcel;
        parcel *p;
        tmpparcel.parcelID = i;
        tmpparcel.weight =  rand() % 10 + 1;
        tmpparcel.priority = 1;
        tmpparcel.exp_time = rand() % 300;
        int n = numdst->intValue();
        int dindex = rand() % n;
        tmpparcel.parceldest = dst[dindex];
        parcel_depot.push_back(tmpparcel);

    }

    // 중복 제거 함수 호출
    //removeDuplicatesInDepot(&parcel_depot);
}

vector<parcel> DroneNetMob::droneParcelsSelectionFromSource(int parcelSel){
    vector<parcel> selectedParcels;
    cout <<"builds working"  << endl;
    cout << " Selection ===>  " << parcelSel << endl;
    cout << " Drone -----> " << getParentModule()->getFullName() <<
                    " with speed = " << speedParameter->doubleValue() <<" Defines its mission!"<< endl;
    if (!isEnd){

        //같은 좌표끼리 그룹
        unordered_map<string, GroupedParcel> parcelGroups;
        for (const auto& p : parcel_depot) {
            string coord = to_string(p.parceldest.x) + "," + to_string(p.parceldest.y);
            parcelGroups[coord].parcels.push_back(p);
            parcelGroups[coord].totalWeight += p.weight;
            parcelGroups[coord].dest = p.parceldest;
        }

        //드론의 무게 제한을 고려하여 택배 선택
        double packedWeight = 0;
        unordered_map<string, vector<parcel>> selectedGroups;

         // 각 그룹에서 가능한 만큼의 택배 선택
        for (auto& group : parcelGroups) {
            vector<parcel>& groupParcels = group.second.parcels;
            vector<parcel> selectedFromGroup;
            
            for (const auto& p : groupParcels) {
                if (packedWeight + p.weight < droneWeightCapacity) {
                    selectedFromGroup.push_back(p);
                    packedWeight += p.weight;
                } else {
                    break;
                }
            }

            if (!selectedFromGroup.empty()) {
                selectedParcels.insert(selectedParcels.end(), 
                    selectedFromGroup.begin(), selectedFromGroup.end());
                
                // 선택된 택배들을 원래 그룹에서 제거
                groupParcels.erase(groupParcels.begin(), 
                    groupParcels.begin() + selectedFromGroup.size());
            }
        }

        //남은 택배들로 parcel_depot 업데이트
        parcel_depot.clear();
        for (const auto& group : parcelGroups) {
            for (const auto& p : group.second.parcels) {
                parcel_depot.push_back(p);
            }
        }

        // 상태 업데이트
        carriedWeight = packedWeight;
        carriedParcels = selectedParcels.size();


        cout << "Number of selected Parcels = " << carriedParcels 
             << "|| Remaining Parcels = " << parcel_depot.size() << endl;

        if (isEnd == false && parcel_depot.size() == 0) {
            isEnd = true;
            cout << "------------------------------------------Last Parcels----------------------------------" << endl;
            totalParcels = carriedParcels + 2;
            cout << "Total Parcels: " << carriedParcels << endl;
        }

        // 선택된 택배 정보 출력
        cout << getParentModule()->getFullName() << " selected parcels:" << endl;
        unordered_map<string, int> destCount;
        for (const auto& p : selectedParcels) {
            string coord = to_string(p.parceldest.x) + "," + to_string(p.parceldest.y);
            destCount[coord]++;
            cout << "ID = " << p.parcelID << " Weight: " << p.weight 
                 << " deadline = " << p.exp_time 
                 << " Destination: (" << p.parceldest.x << "," << p.parceldest.y << ")" << endl;
        }

        // 목적지별 택배 수 출력
        cout << "\nDestination summary:" << endl;
        for (const auto& dest : destCount) {
            cout << "Destination " << dest.first << ": " << dest.second << " parcels" << endl;
        }
    }

    return selectedParcels;
}
Coord DroneNetMob::missionPathNextDest(Coord cpos){
    Coord nextdest;
    Coord c = cpos;
    //test 용

    if (!OngoingMission){
        MissionParcels  = droneParcelsSelectionFromSource(selectionMethod);
        OngoingMission = true;

        sortedParcels = MissionParcels;
        vector<parcel> result;

        switch (selectionMethod) {
            case 0: {
                //Nearest First
                result = NearestFirst(sortedParcels, tspDistance, getMaxSpeed(), batteryConsumption);
                sortedParcels = result;  // 결과를 저장

                //print the destination of the parcels
                // 정렬된 결과 출력
                cout << "\nNearest First Results:" << endl;
                cout << "Total Distance: " << tspDistance << endl;
                cout << "Battery Consumption: " << batteryConsumption << " mAh" << endl;

                // 목적지별 택배 수 출력
                unordered_map<string, int> destCount;
                for(const auto& p : sortedParcels) {
                    string coord = to_string(p.parceldest.x) + "," 
                                 + to_string(p.parceldest.y);
                    destCount[coord]++;
                }

                cout << "\nDelivery Plan:" << endl;
                for(const auto& dest : destCount) {
                    cout << "Destination " << dest.first 
                         << ": " << dest.second << " parcels" << endl;
                }
                break;
            }
            case 1: {
                //Heaviest First
                result = HeaviestFirst(sortedParcels, tspDistance, getMaxSpeed(), batteryConsumption);
                sortedParcels = result;  // 결과를 저장
                //print the destination of the parcels
                for (unsigned int i = 0; i < MissionParcels.size(); i++){
                    cout << " destination of Heaviest Greedy: " <<sortedParcels[i].parceldest.x <<"; "<<sortedParcels[i].parceldest.y <<"; " << endl;
                }
                break;
            }
            case 2: {
                // Dynamic Programming TSP
                // MissionParcels = dp_tsp(MissionParcels, tspDistance, carriedParcels, totalParcels);
                // //print the destination of the parcels
                // for (unsigned int i = 0; i < MissionParcels.size(); i++){
                //     cout << " destination of DP: " <<MissionParcels[i].parceldest.x <<"; "<<MissionParcels[i].parceldest.y <<"; " << endl;
                // }
                // break;

                //Balanced Greedy
                result = efficientTrajectoryDesign(sortedParcels, tspDistance, getMaxSpeed(), batteryConsumption);
                sortedParcels = result;  // 결과를 저장
                //print the destination of the parcels
                for (unsigned int i = 0; i < MissionParcels.size(); i++){
                    cout << " destination of Balanced Greedy: " <<sortedParcels[i].parceldest.x <<"; "<<sortedParcels[i].parceldest.y <<"; " << endl;
                }
                break;
            }
            case 3: {
                // BnB TSP
                remove_dupcoordinates(MissionParcels, carriedParcels, totalParcels);

                MissionParcels = dfs_bnb(MissionParcels, tspDistance, carriedParcels);
                
                //dp_tsp(copiedMissionParcels, carriedParcels);
                //print the destination of the parcels
                for (unsigned int i = 0; i < MissionParcels.size(); i++){
                    cout << " destination of BnB: " <<MissionParcels[i].parceldest.x <<"; "<<MissionParcels[i].parceldest.y <<";  Ind = "<< i << endl;
                }
                break;
            }
            case 4: {
                //Greedy - battery consumption
                MissionParcels = greedyTSP_B(MissionParcels, tspDistance, droneremainingbattery, getMaxSpeed());
                //calculate the battery consumption
                // batteryConsumption = 0;
                cout << "battery capacity: " << droneremainingbattery << endl;
                cout << "Battery consumed: " << batteryConsumption << " mAh" << endl;
                //print the destination of the parcels
                for (unsigned int i = 0; i < MissionParcels.size(); i++){
                    cout << " destination of EEPD: " <<MissionParcels[i].parceldest.x <<"; "<<MissionParcels[i].parceldest.y <<"; " << endl;
                }
                break;
            }
            case 5: {
                MissionParcels = bnb_B(MissionParcels);

                //print the destination of the parcels
                for (unsigned int i = 0; i < MissionParcels.size(); i++){
                    cout << " destination of Energy BnB: " <<MissionParcels[i].parceldest.x <<"; "<<MissionParcels[i].parceldest.y <<"; " << endl;
                }
                break;
            }
            case 6: {
                MissionParcels = efficientTrajectoryDesign(MissionParcels, tspDistance, getMaxSpeed(), batteryConsumption);

                //print the destination of the parcels
                for (unsigned int i = 0; i < MissionParcels.size(); i++){
                    cout << " destination of ETDS: " <<MissionParcels[i].parceldest.x <<"; "<<MissionParcels[i].parceldest.y <<"; " << endl;
                }
                break;
            }
        }
    }
    else{
        
        if (sortedParcels.size() == 0){
            nextdest = originPos;
            OngoingMission = false;
        }
        else{
            double nearestDist = 0;
            int k = 0; //Next Parcel Index
        
            for (unsigned int i = 0; i < sortedParcels.size(); i++){
                double tmpd = sqrt(pow(sortedParcels[i].parceldest.x - cpos.x, 2)
                                    + pow(sortedParcels[i].parceldest.y - cpos.y, 2)
                                    +pow(sortedParcels[i].parceldest.z - cpos.z, 2));
                nextdest = sortedParcels[i].parceldest;
                nearestDist = tmpd;
                k = i;
            }
            sortedParcels.erase(sortedParcels.begin()+k);
        }
    }
    //다음 좌표 출력
    // cout <<" Destination --- > (" <<nextdest.x <<"; " <<nextdest.y <<"; " <<nextdest.z << ")"
    //         <<" Origin ---> (" <<originPos.x <<"; "<<originPos.y << "; " << originPos.z<< ")" << endl;

    totalParcels--;
    if (totalParcels == 0){

        missionTime = tspDistance / speedParameter->doubleValue();


        // record the results
        if(!ifstream("results/record.csv")){
            ofstream resultFile;
            resultFile.open ("results/record.csv");
            resultFile << "Selection Method, Number of Destinations, Distance, Energy, Time" << endl;
        }

        ofstream resultFile;
        resultFile.open ("results/record.csv", ios::app);
        
        //add values
        resultFile << selectionMethod << ", " << numdst->intValue() << ", " << tspDistance << ", " << batteryConsumption << ", " << missionTime << endl;

        resultFile.close();

        cout << "------------------------------------------recorded-----------------------------------" << endl;

        cout << "distance recorded: " << tspDistance << endl;
        cout << "energy recorded: " << batteryConsumption << endl;
        cout << "time recorded: " << missionTime << endl;

    }
    //print drone name and the destination
    cout << getParentModule()->getFullName() << " Destination: " << nextdest.x << ", " << nextdest.y << endl;
    return nextdest;
}

/*Algorithm for destination selection based on:
 * Shortest path and drone capacity*/
Coord DroneNetMob::destAssignment(){
    int sz = dst.size();
    Coord ds = dst[gen];
    gen++;
    return ds;
}



} // namespace inet
