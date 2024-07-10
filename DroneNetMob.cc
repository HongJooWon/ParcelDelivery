/*
 * DroneNetMob.cc
 *
 *  Created on: June 14, 2021
 *      Author: iotlab_aime
 */


#include "inet/common/INETMath.h"
#include "DroneNetMob.h"
#include <bitset>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
namespace inet {

bool flag_original = false;
Coord originPos;
std::vector<Coord> dst; //Destination Positions
int gen = 0;
int nparcels = 100;//what is this for
bool flagArrangedDepot = false;
bool OngoingMission = false;
bool isEnd = false;
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

//greedy algorithm for TSP
vector<parcel> greedyTSP(vector<parcel>& parcels, double& distance){
    vector<parcel> sortedParcels;
    vector<bool> visited(parcels.size(), false);
    Coord currPos = originPos;
    double totalDist = 0;
    while(sortedParcels.size() < parcels.size()) {
        double minDist = 1e9;
        int nextParcel = -1;
        for(int i=0; i<parcels.size(); i++) {
            if(!visited[i]) {
                double d = dist(currPos, parcels[i].parceldest);
                if(d < minDist) {
                    minDist = d;
                    nextParcel = i;
                }
            }
        }
        totalDist += minDist;
        visited[nextParcel] = true;
        sortedParcels.push_back(parcels[nextParcel]);
        currPos = parcels[nextParcel].parceldest;
    }
    totalDist += dist(currPos, originPos);
    cout << "Greedy TSP distance: " << totalDist << endl;
    distance += totalDist;
    cout << "Total distance increased " << distance << endl;
    return sortedParcels;
}

//optimal path가 수행되는지 확인
double tsp(vector<parcel>& parcels, int pos, int visited, vector<vector<double>>& dp, vector<vector<double>>& distance, double currDist, double& ans, vector<int>& optimalPath, vector<int>& path) {
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
                tsp(parcels, i, visited | (1<<i), dp, distance, newDist, ans, optimalPath, path);
                cout << "visited: " << bitset<32>(visited | (1<<i)) << endl;
            }
        }
    }
    dp[pos][visited] = currDist; // update the previously calculated distance
    path.pop_back(); // remove the current position from the path
    return ans;
}

vector<parcel> solveTSP(vector<parcel>& parcels) {
    cout << "Solving TSP" << endl;
    //print the parcels
    for(int i=0; i<parcels.size(); i++) {
        cout << "Parcel to solve" << parcels[i].parcelID << " : " << parcels[i].parceldest.x << " " << parcels[i].parceldest.y << endl;
    }
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
    tsp(parcels, 0, 1, dp, distance, 0, ans, optimalPath, path);

    // Sort parcels based on the optimal path
    vector<parcel> sortedParcels;
    for(int i=optimalPath.size()-1; i>=0; i--) {
        sortedParcels.push_back(parcels[optimalPath[i]]);
    }

    //print the optimal path
    cout << "Optimal path: ";
    for(int i=0; i<optimalPath.size(); i++) {
        cout << optimalPath[i] << " ";
    }
    cout << endl;

    cout << "Minimum distance: " << ans << endl;
    return sortedParcels;
}

void dfs(int start, int next, double value, vector<int>& visited, int n, vector<vector<double>>& travel, double& min_value, vector<int>& path) {
    // 현재까지의 거리가 이미 찾은 최단 거리보다 크다면, 이 경로는 최단 경로가 될 수 없으므로 더 이상 탐색하지 않음
    if(min_value < value)
        return;
    // 모든 택배를 배달했다면, 시작점으로 돌아갈 수 있는지 확인
    if(visited.size() == n) {
        // 시작점으로 돌아갈 수 있다면, 시작점으로 돌아가는 거리를 현재 거리에 더하고, 이 값이 최단 거리보다 짧다면 최단 거리를 업데이트
        if(travel[next][start] != 0) {
            double ori = min_value;
            min_value = min(min_value, value + travel[next][start]);
            // 최단 거리가 업데이트되었다면, 최적 경로도 업데이트
            if(ori != min_value)
                path = visited;
        }
        return;
    }

    // 아직 배달하지 않은 모든 택배를 순회
    for(int i = 0; i < n; i++) {
        // 다음 택배로 이동할 수 있고, 그 택배가 시작점이 아니며, 아직 배달하지 않았다면, 그 택배로 이동
        if(travel[next][i] != 0 && i != start && find(visited.begin(), visited.end(), i) == visited.end()) {
            visited.push_back(i);
            // 다음 택배로 이동하고, 이동하는 데 필요한 거리를 현재 거리에 더하여 DFS를 계속
            dfs(start, i, value + travel[next][i], visited, n, travel, min_value, path);
            // 백트래킹을 위해 방문 리스트에서 마지막 택배를 제거
            visited.pop_back();
        }
    }
}

// BnB Tsp
vector<parcel> dfs_bnb(vector<parcel>& parcels) {

    // 시작점을 (0,0)으로 설정하고 택배 리스트에 추가
    parcel startParcel;
    startParcel.parcelID = 0;
    startParcel.parceldest.x = 0;
    startParcel.parceldest.y = 0;
    parcels.insert(parcels.begin(), startParcel);

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

    // DFS를 시작
    // dfs(0, 0, 0, visited, n, travel, min_value, path);

    // 최적 경로에 따라 택배를 정렬
    vector<parcel> sorted_parcels;
    for(int i : path) {
        sorted_parcels.push_back(parcels[i]);
    }

    // 정렬된 택배 리스트를 반환
    return parcels;
}

DroneNetMob::DroneNetMob()
{
}

void DroneNetMob::initialize(int stage)
{
    LineSegmentsMobilityBase::initialize(stage);

    missionTimeSignal = registerSignal("missionTime");
    tspDistanceSignal = registerSignal("tspDistance");
//    std::cout << "initializing DroneMobility stage " << stage << endl;

//    EV_TRACE << "initializing DroneMobility stage " << stage << endl;
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
            int numParcel = 30;
            parcelsDefinition(numParcel);
/*           for (auto i:parcel_depot){
               std::cout << i.parcelID <<" ; W =" <<i.weight <<"; P = " << i.priority << "; Exp = " <<i.exp_time
                       <<" Pos = (" <<i.parceldest.x <<"; " << i.parceldest.y <<"; " <<i.parceldest.z <<")" << std::endl;
           }*/
        }
/*        destination = missionPathNextDest(originPos);
        std::cout << " Name -----> " << getParentModule()->getFullName() <<
                    " And Speed = " << speedParameter->doubleValue() << std::endl;*/
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
            // if(isEnd == true && timeRecorded == false){
            //     //record the time
            //     timeRecorded = true;
            //     cout << "------------------------------------------time recorded-----------------------------------" << endl;
            //     EV_INFO << "Recorded distance" << tspDistance << endl;
            //     cout << "Recorded distance" << tspDistance << endl;
            // }
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


/*           std::cout << " Name -----> " << getParentModule()->getFullName() << " , Velocity = ("
                            << getCurrentVelocity().x << "; " << getCurrentVelocity().y << "; "<< getCurrentVelocity().z <<"), Position = ("
                            << getCurrentPosition().x<< "; " << getCurrentPosition().y <<"; "<< getCurrentPosition().z <<")"<< std::endl;*/
        }
    }
}

double DroneNetMob::getMaxSpeed() const
{
    return speedParameter->isExpression() ? NaN : speedParameter->doubleValue();
}

void DroneNetMob::destGen(int ndst){
    for (unsigned int i = 0; i < numdst->intValue(); i++){
        Coord nextdst;
        nextdst.x = rand() % 600;
        nextdst.y = rand() % 400;
        nextdst.z = 0;
        dst.push_back(nextdst);
    }
/*    std::cout <<std::endl <<std::endl;
    for(auto i:dst){
        std::cout <<"(" <<i.x <<"; " << i.y <<"; " <<i.z <<") | ";
    }
    std::cout <<std::endl <<std::endl;*/
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
    switch(parcelSel){
        /* Closest-Deadline-Parcel-First
         * Depot sorted by Deadline
         * First deliver the ones with small deadline*/
        case CDPF:{
            std::cout <<" CDPF ----- >    Parcel Selection Method! parcel_depot size = " <<parcel_depot.size() << std::endl;
            if (!flagArrangedDepot){
                std::sort (parcel_depot.begin(), parcel_depot.end(),sortDepotByDeadline);
/*                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    std::cout <<"P" <<parcel_depot[i].parcelID << ":: " << "W = " <<parcel_depot[i].weight <<" dst = (" <<parcel_depot[i].parceldest.x <<"; "<<parcel_depot[i].parceldest.y <<"; "
                            <<parcel_depot[i].parceldest.z <<")"<<" Deadline = "<< parcel_depot[i].exp_time  <<std::endl;
                }*/
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
                std::cout <<"Number of removed Parcels = " <<k<< "|| Remaining Parcels = " <<parcel_depot.size() <<std::endl;
                if(isEnd == false && parcel_depot.size() == 0){
                    isEnd = true;
                    cout << "------------------------------------------Last Parcels----------------------------------" << endl;
                    totalParcels = k + 2; // 2 is for the start and the end
                }
            }
            else{
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
                std::cout <<"22222 Number of removed Parcels = " <<k<<" || Depot = " <<parcel_depot.size()<<std::endl;
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
                std::cout <<"22222 Number of removed Parcels = " <<k<<" || Depot = " <<parcel_depot.size()<<std::endl;
            }
            break;
        }
        /* Closest-Neighbor-Parcel-First
         * Depot sorted by Positions
         * First deliver the ones closer to source*/
        case CNPF:{
            cout <<" CNPF ----- > working------------"  << endl;

            if (!flagArrangedDepot){

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
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            else{
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
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            break;
        }
        /* Efficient Parcel Delivery Service
         * Depot sorted in a greedy way ()
         * First deliver the ones with small ratio distance/weight*/
        case EPDS:{
            if (!flagArrangedDepot){
                std::sort (parcel_depot.begin(), parcel_depot.end(),greedySortDepot);
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
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            else{
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
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            break;
        }
        /* Randomly-Selected-Parcel-First
         * Randomly select Parcels to be delivered first*/
        case RSPF:{
            if (!flagArrangedDepot){

            }
            else{

            }
            break;
        }
        /* Heaviest Parcel First
         * Depot is sorted based on parcel weight
         * First deliver the heaviest ones to the lightest*/
        case HPF:{
            if (!flagArrangedDepot){
                std::sort (parcel_depot.begin(), parcel_depot.end(),SortDepotByWeight);
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
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            else{
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
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            break;
        }
        default:{
            std::cout <<" Undefined Selection Method." <<std::endl;
        }
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
                MissionParcels = greedyTSP(MissionParcels, tspDistance) ;
                //print the destination of the parcels
                for (unsigned int i = 0; i < MissionParcels.size(); i++){
                    cout << " destination of Greedy: " <<MissionParcels[i].parceldest.x <<"; "<<MissionParcels[i].parceldest.y <<"; " << endl;
                }
                break;
            case 1:
                // BnB TSP
                MissionParcels = dfs_bnb(MissionParcels);
                //print the destination of the parcels
                for (unsigned int i = 0; i < MissionParcels.size(); i++){
                    cout << " destination of BnB: " <<MissionParcels[i].parceldest.x <<"; "<<MissionParcels[i].parceldest.y <<"; " << endl;
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
    std::cout <<" Destination --- > (" <<nextdest.x <<"; " <<nextdest.y <<"; " <<nextdest.z << ")"
            <<" Origin ---> (" <<originPos.x <<"; "<<originPos.y << "; " << originPos.z<< ")" << std::endl;

    totalParcels--;
    if (totalParcels == 0){
        missionTime = nextChange;
        cout << "------------------------------------------recorded-----------------------------------" << endl;
        //create csv file with columns: number of destination, distance, time
        ofstream resultFile;
        resultFile.open ("results/record.csv", ios::app);
        
        //add values
        resultFile << selectionMethod << ", " << numdst->intValue() << ", " << tspDistance << ", " << missionTime << endl;



        cout << "distance recorded: " << tspDistance << endl;
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

