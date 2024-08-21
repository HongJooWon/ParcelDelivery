#include "inet/common/INETMath.h"
#include "DroneNetMob.h"
#include <bitset>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unordered_map>
#include <queue>
#include <ctime>

using namespace std;
namespace inet {

bool flag_original = false;
Coord originPos = Coord(0, 0, 0);
vector<Coord> dst; //Destination Positions
int gen = 0;
int nparcels = 100;//what is this for
bool flagArrangedDepot = false;
bool OngoingMission = false;
bool isEnd = false;
int dup = 0;
// int selectionMethod = 0;

vector<parcel> parcel_depot;

Define_Module(DroneNetMob);
bool sortDepotByDeadline (parcel i, parcel j) {
    return (i.exp_time < j.exp_time);
}
bool sortDepotByDestination (parcel i, parcel j) {
    return ((sqrt(pow(i.parceldest.x - originPos.x, 2) + pow(i.parceldest.y - originPos.y, 2)
                 +pow(i.parceldest.z - originPos.z, 2))) < (sqrt(pow(j.parceldest.x - originPos.x, 2)
                    + pow(j.parceldest.y - originPos.y, 2) + pow(j.parceldest.z - originPos.z, 2))));
}
bool greedySortDepot (parcel i, parcel j) {
    return (((sqrt(pow(i.parceldest.x - originPos.x, 2) + pow(i.parceldest.y - originPos.y, 2)
            +pow(i.parceldest.z - originPos.z, 2)))/i.weight) < ((sqrt(pow(j.parceldest.x - originPos.x, 2)
               + pow(j.parceldest.y - originPos.y, 2) + pow(j.parceldest.z - originPos.z, 2)))/j.weight));
}
bool SortDepotByWeight (parcel i, parcel j) {
    return (i.weight > j.weight);
}

double dist(Coord a, Coord b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

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

//greedy algorithm for TSP
vector<parcel> greedyTSP(vector<parcel>& parcels, double& distance){
    vector<parcel> sortedParcels = {};
    vector<bool> visited(parcels.size(), false);
    Coord currPos = originPos;
    double totalDist = 0;
    while(sortedParcels.size() < parcels.size()) {
        double minDist = -1;
        int nextParcel = -1;
        for(int i=0; i<parcels.size(); i++) {
            if(!visited[i]) {
                double d = dist(currPos, parcels[i].parceldest);
                if(d < minDist || minDist == -1) {
                    minDist = d;
                    nextParcel = i;
                }
            }
        }
        cout << "Current Position: " << currPos.x << ", " << currPos.y << " Next position: " << parcels[nextParcel].parceldest.x << ", " << parcels[nextParcel].parceldest.y << " Distance: " << dist(currPos, parcels[nextParcel].parceldest) << endl;
        totalDist += minDist;
        cout  << "minDist: " << minDist << " Total distance: " << totalDist << endl;

        visited[nextParcel] = true;
        sortedParcels.push_back(parcels[nextParcel]);
        currPos = parcels[nextParcel].parceldest;
    }

    totalDist += dist(currPos, originPos); //출발지로 돌아가는 거리
    cout << "Greedy TSP distance: " << totalDist << endl;
    distance += totalDist;
    cout << "Total distance increased " << distance << endl;

    return sortedParcels;
}

//중복 좌표 제거
void remove_dupcoordinates(vector<parcel>& parcels, int& carriedParcels) {
    dup = 0;
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
        }
    }

    cout << "carriedParcels without Duplicates: " << parcels.size() << endl;
}

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

void dp_tsp(vector<parcel>& parcels, int& carriedParcels) {
    cout << "Solving DP" << endl;
    //test
    remove_dupcoordinates(parcels, carriedParcels);

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
    //return sortedParcels;
}

DroneNetMob::DroneNetMob()
{
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
        int np = (&par("npar"))->intValue();
        cout <<" npar = " << np <<endl;
        WATCH(quaternion);
        numdst = &par("ndst");
        ox =&par("initialX");
        oy = &par("initialY");
        oz = &par("initialZ");
        originPos.x = ox->doubleValue();
        originPos.y = oy->doubleValue();
        originPos.z = oz->doubleValue();
        droneweightcapacity =  (&par("weightCapacity"))->doubleValue();
        droneremainingbattery =  (&par("remainingBattery"))->doubleValue();
        selectionMethod = (&par("parcelSelectionMethod"))->intValue();
        if (strcmp(getParentModule()->getFullName(), "drone[0]") == 0){
            cout << " Name -----> " << getParentModule()->getFullName() <<endl;
            //print selectionmethod
            cout << " Selection Method -----> " << selectionMethod <<endl;
            if (numdst->intValue() > 0){
                destGen (numdst->intValue());
            }

        //택배 개수 할당
        parcelsDefinition(np);

        }

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
            EV_INFO << "new target position = " << targetPosition << ", next change = " << nextChange << endl;
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
            EV_INFO << "new target position = " << targetPosition << ", next change = " << nextChange << endl;

            lastVelocity = (targetPosition - lastPosition) / (nextChange - simTime()).dbl();
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
            cout <<"Vel = " << lastVelocity.x <<"  ; " << lastVelocity.x << "  ; " << lastVelocity.z << endl;
        }
        else if (now > lastUpdate) {
            ASSERT(nextChange == -1 || now < nextChange);
            double alpha = (now - previousChange) / (nextChange - previousChange);
            lastPosition = sourcePosition * (1 - alpha) + targetPosition * alpha;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
            if (strcmp(getParentModule()->getFullName(), "drone[1]") == 0){
                cout <<"Vel = " << lastVelocity.x <<"  ; " << lastVelocity.x << "  ; " << lastVelocity.z << endl;
            }

        }
    }
}

double DroneNetMob::getMaxSpeed() const
{
    return speedParameter->isExpression() ? NaN : speedParameter->doubleValue();
}

void DroneNetMob::destGen(int ndst){
    srand(time(0)); // 현재 시간을 시드로 사용하여 난수 생성기 초기화
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
}
vector<parcel> DroneNetMob::droneParcelsSelectionFromSource(int parcelSel){
    vector<parcel> selectedParcels;
    double packedweight = 0;
    cout <<"builds working"  << endl;
    cout << " Selection ===>  " << parcelSel << endl;
    cout << " Drone -----> " << getParentModule()->getFullName() <<
                    " with speed = " << speedParameter->doubleValue() <<" Defines its mission!"<< endl;
    if (!isEnd){

        cout << "--------------sorted destination list----------------" << endl;
        // sort (parcel_depot.begin(), parcel_depot.end(),sortDepotByDestination);
        for (unsigned int i = 0; i < parcel_depot.size(); i++){
            cout << " destination of parcels in depot: " <<parcel_depot[i].parceldest.x <<"; "<<parcel_depot[i].parceldest.y <<"; " << endl;
        }
        int k=0;
        for (unsigned int i = 0; i < parcel_depot.size(); i++){
            packedweight+=parcel_depot[i].weight;
            if (packedweight < droneweightcapacity){
                selectedParcels.push_back(parcel_depot[i]);
                k++;
            }
            else{
                break;
            }
        }
        cout <<"Remaining Parcels = " <<parcel_depot.size() <<endl;
        parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
        carriedParcels = k;
        cout <<"Number of removed Parcels = " <<k<< "|| Remaining Parcels = " <<parcel_depot.size() <<endl;
        if(isEnd == false && parcel_depot.size() == 0){
            isEnd = true;
            cout << "------------------------------------------Last Parcels----------------------------------" << endl;
            totalParcels = k + 2; // 2 is for the start and the end

            cout << "Total Parcels: " << carriedParcels << endl;
        }
    }
    else{
        // int k=0;
        // for (unsigned int i = 0; i < parcel_depot.size(); i++){
        //     packedweight+=parcel_depot[i].weight;
        //     if (packedweight < droneweightcapacity){
        //         selectedParcels.push_back(parcel_depot[i]);
        //         k++;
        //     }
        //     else{
        //         break;
        //     }
        // }
        // parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
    }
    for (auto i:selectedParcels){
        cout <<"+++ ID = " << i.parcelID << " Weight: " <<i.weight <<" deadline = " << i.exp_time <<
                "   <<-->> parcel_depot Size = "<< parcel_depot.size() <<endl;
    }

    return selectedParcels;
}
Coord DroneNetMob::missionPathNextDest(Coord cpos){
    Coord nextdest;
    if (!OngoingMission){
        MissionParcels  = droneParcelsSelectionFromSource(selectionMethod);
        OngoingMission = true;

        //test 용
        vector<parcel> copiedMissionParcels = MissionParcels;

        switch (selectionMethod) {
            case 0:
                //Greedy
                MissionParcels = greedyTSP(MissionParcels, tspDistance);
                //dfs_bnb(MissionParcels, tspDistance, totalParcels);
                //print the destination of the parcels
                for (unsigned int i = 0; i < MissionParcels.size(); i++){
                    cout << " destination of Greedy: " <<MissionParcels[i].parceldest.x <<"; "<<MissionParcels[i].parceldest.y <<"; " << endl;
                }
                break;
            case 1:
                // BnB TSP
                remove_dupcoordinates(MissionParcels, carriedParcels);

                MissionParcels = dfs_bnb(MissionParcels, tspDistance, carriedParcels);
                
                dp_tsp(copiedMissionParcels, carriedParcels);
                //print the destination of the parcels
                for (unsigned int i = 0; i < MissionParcels.size(); i++){
                    cout << " destination of BnB: " <<MissionParcels[i].parceldest.x <<"; "<<MissionParcels[i].parceldest.y <<";  Ind = "<< i << endl;
                }
                break;
            case 2:
                // Dynamic Programming TSP
                //MissionParcels = dp_tsp(MissionParcels);
                //print the destination of the parcels
                for (unsigned int i = 0; i < MissionParcels.size(); i++){
                    cout << " destination of DP: " <<MissionParcels[i].parceldest.x <<"; "<<MissionParcels[i].parceldest.y <<"; " << endl;
                }
                break;
        }
    }
    else{
        if (MissionParcels.size() == 0){
            nextdest = originPos;
            OngoingMission = false;
        }
        else{
            double nearestDist = 0;
            int k = 0; //Next Parcel Index
        
            for (unsigned int i = 0; i < MissionParcels.size(); i++){
                if (i == 0){
                    double tmpd = sqrt(pow(MissionParcels[i].parceldest.x - cpos.x, 2)
                                        + pow(MissionParcels[i].parceldest.y - cpos.y, 2)
                                        +pow(MissionParcels[i].parceldest.z - cpos.z, 2));
                    nextdest = MissionParcels[i].parceldest;
                    nearestDist = tmpd;
                    k = i;
                }
                else{
                    double tmpd = sqrt(pow(MissionParcels[i].parceldest.x - cpos.x, 2)
                                        + pow(MissionParcels[i].parceldest.y - cpos.y, 2)
                                        +pow(MissionParcels[i].parceldest.z - cpos.z, 2));
                    if (tmpd < nearestDist){
                        nextdest = MissionParcels[i].parceldest;
                        nearestDist = tmpd;
                        k = i;
                    }
                }
            }
            MissionParcels.erase(MissionParcels.begin()+k);
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
            resultFile << "Selection Method, Number of Destinations, Distance, Time" << endl;
        }

        ofstream resultFile;
        resultFile.open ("results/record.csv", ios::app);
        
        //add values
        resultFile << selectionMethod << ", " << numdst->intValue() << ", " << tspDistance << ", " << missionTime << endl;

        resultFile.close();

        cout << "------------------------------------------recorded-----------------------------------" << endl;

        cout << "distance recorded: " << tspDistance << endl;
        cout << "time recorded: " << missionTime << endl;

        //print parcel list
        for (unsigned int i = 0; i < MissionParcels.size(); i++){
            cout << "Parcel ID: " << MissionParcels[i].parcelID << " Parcel Destination: " << MissionParcels[i].parceldest.x << ", " << MissionParcels[i].parceldest.y << endl;
        }
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