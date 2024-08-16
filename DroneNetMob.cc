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
std::vector<Coord> dst; //Destination Positions
int gen = 0;
int nparcels = 100;//what is this for
bool flagArrangedDepot = false;
bool OngoingMission = false;
bool isEnd = false;
int dup = 0;
// int selectionMethod = 0;

std::vector<parcel> parcel_depot;

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
    vector<double> key(n, numeric_limits<double>::infinity());
    vector<bool> mstSet = visited;
    
    // 시작 노드를 제외하고 MST 계산을 위한 초기화
    for (int i = 0; i < n; i++) {
        if (i != start && !visited[i]) {
            key[i] = travel[start][i];
        }
    }
    
    // Prim's 알고리즘을 사용하여 MST 계산
    for (int count = 0; count < n - 1; count++) {
        int u = -1;
        double min = numeric_limits<double>::infinity();
        for (int v = 0; v < n; v++) {
            if (v != start && !mstSet[v] && key[v] < min) {
                min = key[v];
                u = v;
            }
        }
        
        if (u == -1) break;  // 모든 노드가 방문되었거나 연결할 수 없는 경우
        
        mstSet[u] = true;
        
        for (int v = 0; v < n; v++) {
            if (v != start && !mstSet[v] && travel[u][v] < key[v]) {
                key[v] = travel[u][v];
            }
        }
    }
    
    // MST의 총 가중치 계산
    double mstCost = 0;
    for (int i = 0; i < n; i++) {
        if (i != start && !visited[i]) {
            mstCost += key[i];
        }
    }
    
    // 시작 노드에서 가장 가까운 두 노드 찾기
    vector<double> minEdges;
    for (int i = 0; i < n; i++) {
        if (i != start && !visited[i]) {
            minEdges.push_back(travel[start][i]);
        }
    }
    sort(minEdges.begin(), minEdges.end());
    
    // 1-tree cost 계산: MST 가중치 + 시작 노드와 연결된 가장 짧은 두 엣지
    double oneTreeCost = mstCost;
    if (minEdges.size() >= 2) {
        oneTreeCost += minEdges[0] + minEdges[1];
    }
    
    return oneTreeCost;
}

void dfs(int start, int next, double value, vector<int>& visited, int n, vector<vector<double>>& travel, double& min_value, vector<int>& path) {
    // 현재까지 방문한 노드 표시
    vector<bool> is_visited(n, false);
    for (int v : visited) is_visited[v] = true;

    // 1-tree lower bound 계산
    double lower_bound = value + calculate1TreeLowerBound(travel, next, is_visited);
    
    // 가지치기: 현재 경로의 lower bound가 지금까지의 최소값보다 크거나 같으면 탐색 중단
    if (lower_bound >= min_value) {
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
    startParcel.parceldest.x = 0;
    startParcel.parceldest.y = 0;
    parcels.insert(parcels.begin(), startParcel);
    // 택배 리스트를 출력
    
    for(int i=0; i<parcels.size(); i++) {
        cout << "Parcel to solve" << parcels[i].parcelID << " : " << parcels[i].parceldest.x << " " << parcels[i].parceldest.y << endl;
    }

    remove_dupcoordinates(parcels, carriedParcels);

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

    // 최적 경로에 따라 택배를 정렬
    vector<parcel> sorted_parcels;
    for(int i : path) {
        sorted_parcels.push_back(parcels[i]);
    }

    //print the optimal path
    cout << "Optimal path: ";
    for(int i=0; i<path.size(); i++) {
        cout << path[i] << " ";
    }
    
    //print the sorted parcels
    cout << "Sorted parcels: ";
    for(int i=0; i<sorted_parcels.size(); i++) {
        cout << sorted_parcels[i].parcelID << " ";
    }
    cout << endl;

    cout << "Minimum distance: " << min_value << endl;
    distance += min_value;
    cout << "BnB total distance increased: " << distance << endl;

    // 정렬된 택배 리스트를 반환
    return sorted_parcels;
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
        cout <<" npar = " << np <<std::endl;
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
        if (std::strcmp(getParentModule()->getFullName(), "drone[0]") == 0){
            std::cout << " Name -----> " << getParentModule()->getFullName() <<std::endl;
            //print selectionmethod
            std::cout << " Selection Method -----> " << selectionMethod <<std::endl;
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
            std::cout <<"Vel = " << lastVelocity.x <<"  ; " << lastVelocity.x << "  ; " << lastVelocity.z << std::endl;
        }
        else if (now > lastUpdate) {
            ASSERT(nextChange == -1 || now < nextChange);
            double alpha = (now - previousChange) / (nextChange - previousChange);
            lastPosition = sourcePosition * (1 - alpha) + targetPosition * alpha;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
            if (std::strcmp(getParentModule()->getFullName(), "drone[1]") == 0){
                std::cout <<"Vel = " << lastVelocity.x <<"  ; " << lastVelocity.x << "  ; " << lastVelocity.z << std::endl;
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
std::vector<parcel> DroneNetMob::droneParcelsSelectionFromSource(int parcelSel){
    std::vector<parcel> selectedParcels;
    double packedweight = 0;
    cout <<"builds working"  << endl;
    std::cout << " Selection ===>  " << parcelSel << std::endl;
    std::cout << " Drone -----> " << getParentModule()->getFullName() <<
                    " with speed = " << speedParameter->doubleValue() <<" Defines its mission!"<< std::endl;
    if (!isEnd){

        cout << "--------------sorted destination list----------------" << endl;
        // std::sort (parcel_depot.begin(), parcel_depot.end(),sortDepotByDestination);
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
        std::cout <<"Remaining Parcels = " <<parcel_depot.size() <<std::endl;
        parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
        carriedParcels = k;
        std::cout <<"Number of removed Parcels = " <<k<< "|| Remaining Parcels = " <<parcel_depot.size() <<std::endl;
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
        std::cout <<"+++ ID = " << i.parcelID << " Weight: " <<i.weight <<" deadline = " << i.exp_time <<
                "   <<-->> parcel_depot Size = "<< parcel_depot.size() <<std::endl;
    }

    return selectedParcels;
}
Coord DroneNetMob::missionPathNextDest(Coord cpos){
    Coord nextdest;
    if (!OngoingMission){
        MissionParcels  = droneParcelsSelectionFromSource(selectionMethod);
        OngoingMission = true;

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
                MissionParcels = dfs_bnb(MissionParcels, tspDistance, carriedParcels);
                //print the destination of the parcels
                for (unsigned int i = 0; i < MissionParcels.size(); i++){
                    cout << " destination of BnB: " <<MissionParcels[i].parceldest.x <<"; "<<MissionParcels[i].parceldest.y <<";  Ind = "<< i << endl;
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
    // std::cout <<" Destination --- > (" <<nextdest.x <<"; " <<nextdest.y <<"; " <<nextdest.z << ")"
    //         <<" Origin ---> (" <<originPos.x <<"; "<<originPos.y << "; " << originPos.z<< ")" << std::endl;

    totalParcels--;
    if (totalParcels == 0){

        missionTime = tspDistance / speedParameter->doubleValue();


        // record the results
        if(!std::ifstream("results/record.csv")){
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