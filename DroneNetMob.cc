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
int parcelNum = 0;

double weight_a = 0;  // 무게 가중치
double weight_b = 0;   // 거리 가중치


struct DeliveryState {
    vector<int> path;           // 현재까지의 경로 (그룹 인덱스)
    vector<parcel> selectedParcels;  // 각 그룹에서 선택된 택배들
    double currentCost;         // 현재까지의 비용
    double currentWeight;       // 현재 적재 무게
    double bound;              // 하한값
    int level;                 // 탐색 깊이
};

struct CompareState {
    bool operator()(const DeliveryState& s1, const DeliveryState& s2) {
        return s1.bound > s2.bound;
    }
};

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
            //energy += batteryCalculation(currPos, selectedGroup.dest, selectedGroup.totalWeight, speed);
            
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
    //energy += batteryCalculation(currPos, originPos, 0, speed); //출발지로 돌아가는 배터리 소모까지 계산
    //distance += totalDist;
    // cout << "Heaviest-Greedy TSP distance: " << distance << endl;
    // cout << "Heaviest-Greedy TSP energy: " << energy << endl;

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

vector<parcel> BalanceFirst(unordered_map<string, GroupedParcel>& parcelGroups, double& totalDistance, double speed, double& energy, double& carriedWeight, double& droneWeightCapacity) {
    vector<parcel> selectedParcels;
    Coord currentPos = originPos;
    double currentWeight = 0;
    double currentEnergy = 0;
    double currentDistance = 0;

    // 처리할 그룹들의 포인터 벡터 생성
    vector<GroupedParcel*> remainingGroups;
    for (auto& group : parcelGroups) {
        remainingGroups.push_back(&group.second);
    }

    while (!remainingGroups.empty() && currentWeight < droneWeightCapacity) {
        vector<pair<double, pair<GroupedParcel*, parcel>>> scoredParcels;
        double totalWeight = 0;
        double maxDistance = 0;

        // 모든 그룹의 전체 무게와 최대 거리 계산
        for (auto* group : remainingGroups) {
            totalWeight += group->totalWeight;
            double distance = dist(currentPos, group->dest);
            maxDistance = max(maxDistance, distance);
        }

        // 각 그룹 내 개별 택배에 대한 점수 계산
        for (auto* group : remainingGroups) {
            for (const auto& p : group->parcels) {
                if (currentWeight + p.weight <= droneWeightCapacity) {
                    // 무게와 거리 정규화
                    double w_i = p.weight / totalWeight;
                    double d_i = dist(currentPos, p.parceldest) / maxDistance;
                    
                    double ALPHA = weight_a;  // 무게 가중치
                    double BETA = weight_b;   // 거리 가중치
                    // 무게는 높을수록, 거리는 짧을수록 좋은 점수
                    double score = (ALPHA * w_i + BETA * (1.0 - d_i));
                    
                    scoredParcels.push_back({score, {group, p}});
                }
            }
        }

        if (scoredParcels.empty()) {
            cout << "No more parcels can be added within weight limit." << endl;
            break;
        }

        // 점수 기준으로 정렬 (내림차순)
        sort(scoredParcels.begin(), scoredParcels.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });

        // 최고 점수 택배 선택 및 적재
        auto& bestChoice = scoredParcels[0];
        auto* parentGroup = bestChoice.second.first;
        auto& bestParcel = bestChoice.second.second;

        // 거리와 에너지 계산
        currentDistance += dist(currentPos, bestParcel.parceldest);
        currentEnergy += batteryCalculation(currentPos, bestParcel.parceldest, bestParcel.weight, speed);
        
        selectedParcels.push_back(bestParcel);
        currentWeight += bestParcel.weight;
        currentPos = bestParcel.parceldest;

        cout << "\nSelected parcel at (" << bestParcel.parceldest.x << ", " 
             << bestParcel.parceldest.y << "):" << endl;
        cout << "Parcel ID: " << bestParcel.parcelID << endl;
        cout << "Parcel weight: " << bestParcel.weight << endl;
        cout << "Score: " << bestChoice.first << endl;
        cout << "Current total weight: " << currentWeight << "/" << droneWeightCapacity << endl;
        cout << "Current distance: " << currentDistance << endl;

        // 선택된 택배를 그룹에서 제거
        parentGroup->parcels.erase(
            remove_if(parentGroup->parcels.begin(), parentGroup->parcels.end(),
                [&bestParcel](const parcel& p) { return p.parcelID == bestParcel.parcelID; }),
            parentGroup->parcels.end()
        );

        // 그룹이 비었으면 제거
        if (parentGroup->parcels.empty()) {
            remainingGroups.erase(
                remove(remainingGroups.begin(), remainingGroups.end(), parentGroup),
                remainingGroups.end()
            );
        }
    }

    // 출발지로 돌아가는 거리와 에너지 추가
    if (!selectedParcels.empty()) {
        currentDistance += dist(currentPos, originPos);
        currentEnergy += batteryCalculation(currentPos, originPos, 0, speed);
    }

    carriedWeight = currentWeight;

    return selectedParcels;
}

//----------------------------------Ratio-and-angle-based Search-----------------------------------------------

// vector<parcel> RatioAngleBasedSearch(unordered_map<string, GroupedParcel>& parcelGroups, 
//     double& totalDistance, double speed, double& energy, double& carriedWeight, 
//     double& droneWeightCapacity) {
    
//     vector<parcel> selectedParcels;
//     Coord currentPos = originPos;
//     double currentWeight = 0;
//     const double MAX_ANGLE = M_PI/12; // 15도

//     vector<GroupedParcel*> remainingGroups;
//     for (auto& group : parcelGroups) {
//         remainingGroups.push_back(&group.second);
//     }

//     while (!remainingGroups.empty() && currentWeight < droneWeightCapacity) {
//         vector<pair<double, pair<GroupedParcel*, parcel>>> scoredParcels;
        
//         // 각 그룹의 택배들에 대해 ratio와 angle 계산
//         for (auto* group : remainingGroups) {
//             for (const auto& p : group->parcels) {
//                 if (currentWeight + p.weight <= droneWeightCapacity) {
//                     // 거리/무게 비율 계산
//                     double distance = dist(currentPos, p.parceldest);
//                     double ratio = distance / p.weight;
                    
//                     // 각도 계산
//                     double angle = calculateAngle(currentPos, p.parceldest);
                    
//                     // 각도가 허용 범위 내인 경우만 고려
//                     if (abs(angle) <= MAX_ANGLE) {
//                         double score = ratio; // 낮을수록 좋음
//                         scoredParcels.push_back({score, {group, p}});
//                     }
//                 }
//             }
//         }

//         if (scoredParcels.empty()) {
//             break;
//         }

//         // ratio 기준으로 정렬 (오름차순)
//         sort(scoredParcels.begin(), scoredParcels.end());

//         // 최적의 택배 선택
//         auto& bestChoice = scoredParcels[0];
//         auto* parentGroup = bestChoice.second.first;
//         auto& bestParcel = bestChoice.second.second;

//         selectedParcels.push_back(bestParcel);
//         currentWeight += bestParcel.weight;
//         currentPos = bestParcel.parceldest;

//         // 선택된 택배와 빈 그룹 제거
//         updateGroups(remainingGroups, parentGroup, bestParcel);
//     }

//     carriedWeight = currentWeight;
//     return selectedParcels;
// }

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
        weight_a = (&par("alpha_val"))->doubleValue();
        weight_b = (&par("beta_val"))->doubleValue();

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
                tspDistance += dist(lastPosition, targetPosition);

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
    if (isEnd) return selectedParcels;

    // 1. 좌표별로 기본 그룹화
    unordered_map<string, GroupedParcel> parcelGroups;
    
    for (const auto& p : parcel_depot) {
        string coord = to_string(p.parceldest.x) + "," + to_string(p.parceldest.y);
        parcelGroups[coord].parcels.push_back(p);
        parcelGroups[coord].totalWeight += p.weight;
        parcelGroups[coord].dest = p.parceldest;
    }

    // 2. 선택된 방법에 따라 택배 선택
    double currentWeight = 0;
    double currentDistance = 0;
    
    switch (parcelSel) {
        case 0: { // Nearest First
            Coord currentPos = originPos;
            bool canAddMore = true;
            int parcelNum = 0;
            
            // 모든 택배를 하나의 벡터로 변환
            vector<parcel*> remainingParcels;
            for (auto& group : parcelGroups) {
                for (auto& p : group.second.parcels) {
                    remainingParcels.push_back(&p);
                }
            }

            // 무게 제한에 도달할 때까지 반복
            while (!remainingParcels.empty() && canAddMore) {
                // 현재 위치에서 가장 가까운 택배 찾기
                auto nearest = min_element(remainingParcels.begin(), remainingParcels.end(),
                    [currentPos](const parcel* a, const parcel* b) {
                        return dist(currentPos, a->parceldest) < dist(currentPos, b->parceldest);
                    });

                if (nearest != remainingParcels.end()) {
                    parcel* nearestParcel = *nearest;
                    
                    // 무게 제한 확인
                    if (currentWeight + nearestParcel->weight <= droneWeightCapacity) {
                        selectedParcels.push_back(*nearestParcel);
                        currentWeight += nearestParcel->weight;

                        parcelNum++;
                        
                        // 선택된 택배 정보 출력
                        cout << "Parcel " << parcelNum << " selected:" << endl;
                        cout << "  ID: " << nearestParcel->parcelID << endl;
                        cout << "  Weight: " << nearestParcel->weight << endl;
                        cout << "  Destination: (" << nearestParcel->parceldest.x 
                            << ", " << nearestParcel->parceldest.y << ")" << endl;
                        cout << "  Distance from current pos: " 
                            << dist(currentPos, nearestParcel->parceldest) << endl;
                        cout << "  Current total weight: " << currentWeight << "/"
                            << droneWeightCapacity << endl;
                        
                        // 현재 위치 업데이트
                        currentPos = nearestParcel->parceldest;
                        
                        // 선택된 택배 제거
                        remainingParcels.erase(nearest);
                    } else {
                        // 더 이상 택배를 추가할 수 없음
                        canAddMore = false;
                        cout << "Weight capacity reached. Cannot add more parcels." << endl;
                    }
                }
            }

            //돌아오는 거리 추가
            currentDistance += dist(currentPos, originPos);

            tspDistance += currentDistance;
            carriedWeight = currentWeight;
            break;
        }

        case 1: { // Heaviest First
            int parcelNum = 0;
            bool canAddMore = true;  // Add flag for weight limit check
            
            // 그룹별 총 무게를 기준으로 정렬
            vector<GroupedParcel*> groupsByWeight;
            for (auto& group : parcelGroups) {
                groupsByWeight.push_back(&group.second);
            }
            
            // 그룹의 총 무게를 기준으로 정렬 (내림차순)
            sort(groupsByWeight.begin(), groupsByWeight.end(),
                [](const GroupedParcel* a, const GroupedParcel* b) {
                    return a->totalWeight > b->totalWeight;
                });

            // 무거운 그룹부터 처리
            for (const GroupedParcel* group : groupsByWeight) {
                if (!canAddMore) break;  // 무게 제한에 도달하면 중단

                cout << "\nProcessing destination (" << group->dest.x << ", " 
                    << group->dest.y << "):" << endl;
                cout << "Group total weight: " << group->totalWeight << endl;
                
                // 현재 그룹의 택배들을 무게 순으로 정렬
                vector<parcel> sortedParcels = group->parcels;
                sort(sortedParcels.begin(), sortedParcels.end(),
                    [](const parcel& a, const parcel& b) {
                        return a.weight > b.weight;
                    });

                bool addedFromCurrentGroup = false;
                // 무게 제한 내에서 현재 그룹의 택배들을 선택
                for (const auto& p : sortedParcels) {
                    if (currentWeight + p.weight <= droneWeightCapacity) {
                        selectedParcels.push_back(p);
                        currentWeight += p.weight;
                        parcelNum++;
                        addedFromCurrentGroup = true;
                        
                        // 선택된 택배 정보 출력
                        cout << "  Parcel " << parcelNum << " selected:" << endl;
                        cout << "    ID: " << p.parcelID << endl;
                        cout << "    Weight: " << p.weight << endl;
                        cout << "    Current total weight: " << currentWeight << "/"
                            << droneWeightCapacity << endl;
                    } else {
                        // 현재 택배를 추가할 수 없는 경우
                        if (!addedFromCurrentGroup) {
                            // 현재 그룹에서 아무것도 추가하지 못했다면 더 이상 진행 불가
                            canAddMore = false;
                            cout << "  Cannot add any parcels from this group due to weight limit" << endl;
                            break;
                        } else {
                            // 현재 그룹에서 일부는 추가했지만 더 이상 추가할 수 없는 경우
                            cout << "  Cannot add more parcels from this destination" << endl;
                            break;
                        }
                    }
                }

                if (currentWeight >= droneWeightCapacity) {
                    cout << "\nWeight capacity reached. Cannot process more destinations." << endl;
                    canAddMore = false;
                    break;
                }
            }
            
            carriedWeight = currentWeight;
            break;
        }

        case 2: { // Balance First
            vector<parcel> result = BalanceFirst(parcelGroups, tspDistance, getMaxSpeed(), batteryConsumption, carriedWeight, droneWeightCapacity);
            selectedParcels = result;

            break;
        }

        case 3: { // Ratio and Angle Based Search
            // vector<parcel> result = RatioAngleBasedSearch(parcelGroups, tspDistance, getMaxSpeed(), batteryConsumption, carriedWeight, droneWeightCapacity);
            // selectedParcels = result;

            // break;
        }
        
    }

    // 최종 선택 결과 출력
    cout << "\nFinal Selection Summary:" << endl;
    cout << "Total parcels selected: " << selectedParcels.size() << endl;
    cout << "Total weight: " << carriedWeight << "/" << droneWeightCapacity << endl;
    
    // 목적지별 통계
    unordered_map<string, int> destStats;
    for (const auto& p : selectedParcels) {
        string coord = to_string(p.parceldest.x) + "," + to_string(p.parceldest.y);
        destStats[coord]++;
    }
    
    cout << "\nDestination Statistics:" << endl;
    for (const auto& stat : destStats) {
        cout << "Destination " << stat.first << ": " 
            << stat.second << " parcels" << endl;
    }

    //선택된 택배들을 depot에서 제거
    for (const auto& selected : selectedParcels) {
        parcel_depot.erase(
            remove_if(parcel_depot.begin(), parcel_depot.end(),
                [&selected](const parcel& p) {
                    return p.parcelID == selected.parcelID;
                }),
            parcel_depot.end()
        );
    }

    // 상태 업데이트
    //carriedWeight = currentWeight;
    carriedParcels = selectedParcels.size();

    // 모든 택배가 선택되었는지 확인
    if (parcel_depot.empty()) {
        isEnd = true;
        totalParcels = carriedParcels + 2;
    }

    return selectedParcels;
}

Coord DroneNetMob::missionPathNextDest(Coord cpos){
    Coord nextdest;

    if (!OngoingMission) {
        // 택배 선택 및 정렬이 모두 droneParcelsSelectionFromSource에서 수행됨
        MissionParcels = droneParcelsSelectionFromSource(selectionMethod);
        OngoingMission = true;
        sortedParcels = MissionParcels;
    }
    
    // 다음 목적지 선택
    if (sortedParcels.empty()) {
        nextdest = originPos;
        OngoingMission = false;
    } else {
        nextdest = sortedParcels[0].parceldest;

        sortedParcels.erase(sortedParcels.begin());
        
        // 배터리 소모량 계산
        batteryConsumption += batteryCalculation(cpos, nextdest, carriedWeight, getMaxSpeed());
        carriedWeight -= sortedParcels[0].weight;  // 배달된 택배 무게만큼 감소
        
        cout << getParentModule()->getFullName() <<": next carriedWeight: " << carriedWeight << endl;
    }

    // 미션 완료 체크 및 통계 기록
    totalParcels--; 
    if (totalParcels == 0) {
        missionTime = tspDistance / speedParameter->doubleValue();


        // 파일명에 weight_a와 weight_b 포함
        stringstream filename;
        filename << "results/record_weight_" << weight_a << "_" << weight_b << ".csv";
        
        // 파일이 존재하지 않으면 헤더 생성
        if(!ifstream(filename.str())){
            ofstream resultFile;
            resultFile.open(filename.str());
            resultFile << "Selection Method, Number of Destinations, Distance, Energy" << endl;
        }

        // 결과 추가
        ofstream resultFile;
        resultFile.open(filename.str(), ios::app);
        
        //add values
        resultFile << selectionMethod << ", " << numdst->intValue() << ", " << tspDistance << ", " << batteryConsumption << endl;

        resultFile.close();

        cout << "------------------------------------------recorded-----------------------------------" << endl;

        cout << "distance recorded: " << tspDistance << endl;
        cout << "energy recorded: " << batteryConsumption << endl;
        cout << "time recorded: " << missionTime << endl;
    }

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
