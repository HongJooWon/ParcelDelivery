///*
// * DroneNetMob.cc
// *
// *  Created on: June 14, 2021
// *      Author: iotlab_aime
// */
//
//
//#include "inet/common/INETMath.h"
//#include "DroneNetMob.h"
//
//using namespace std;
//namespace inet {
//
//bool flag_original = false;
//Coord originPos;
//std::vector<Coord> dst; //Destination Positions
//int gen = 0;
//int nparcels = 500;
//bool flagArrangedDepot = false;
//bool OngoingMission = false;
//
//std::vector<parcel> parcel_depot;
//
//Define_Module(DroneNetMob);
//bool sortDepotByDeadline (parcel i, parcel j) {
//    return (i.exp_time < j.exp_time);
//}
//bool sortDepotByDestination (parcel i, parcel j) {
//    return ((sqrt(pow(i.parceldest.x - originPos.x, 2) + pow(i.parceldest.y - originPos.y, 2)
//                 +pow(i.parceldest.z - originPos.z, 2))) < (sqrt(pow(j.parceldest.x - originPos.x, 2)
//                    + pow(j.parceldest.y - originPos.y, 2) + pow(j.parceldest.z - originPos.z, 2))));
//}
//bool greedySortDepot (parcel i, parcel j) {
//    return (((sqrt(pow(i.parceldest.x - originPos.x, 2) + pow(i.parceldest.y - originPos.y, 2)
//            +pow(i.parceldest.z - originPos.z, 2)))/i.weight) < ((sqrt(pow(j.parceldest.x - originPos.x, 2)
//               + pow(j.parceldest.y - originPos.y, 2) + pow(j.parceldest.z - originPos.z, 2)))/j.weight));
//}
//bool SortDepotByWeight (parcel i, parcel j) {
//    return (i.weight > j.weight);
//}
//
////path optimization through branch and bound accroding to the distance
//// struct parcel{
////     int parcelID;
////     double weight;
////     int priority;
////     double exp_time;
////     Coord parceldest;
//// };
//
////std::vector<parcel> parcel_depot;
//
//double dist(Coord a, Coord b) {
//    double dx = a.x - b.x;
//    double dy = a.y - b.y;
//    return sqrt(dx*dx + dy*dy);
//}
//
//DroneNetMob::DroneNetMob()
//{
//}
//
//void DroneNetMob::initialize(int stage)
//{
//    LineSegmentsMobilityBase::initialize(stage);
////    std::cout << "initializing DroneMobility stage " << stage << endl;
//
////    EV_TRACE << "initializing DroneMobility stage " << stage << endl;
//    if (stage == INITSTAGE_LOCAL) {
//        rad heading = deg(par("initialMovementHeading"));
//        rad elevation = deg(par("initialMovementElevation"));
//        changeIntervalParameter = &par("changeInterval");
//        angleDeltaParameter = &par("angleDelta");
//        rotationAxisAngleParameter = &par("rotationAxisAngle");
//        speedParameter = &par("speed");
//        quaternion = Quaternion(EulerAngles(heading, -elevation, rad(0)));
//        WATCH(quaternion);
//        numdst = &par("ndst");
//        ox =&par("initialX");
//        oy = &par("initialY");
//        oz = &par("initialZ");
//        originPos.x = ox->doubleValue();
//        originPos.y = oy->doubleValue();
//        originPos.z = oz->doubleValue();
//        droneweightcapacity =  (&par("weightCapacity"))->doubleValue();
//        droneremainingbattery =  (&par("remainingBattery"))->doubleValue();
//        selectionMethod = (&par("parcelSelecitionMethod"))->intValue();
//        if (std::strcmp(getParentModule()->getFullName(), "drone[0]") == 0){
//            std::cout << " Name -----> " << getParentModule()->getFullName() <<std::endl;
//            if (numdst->intValue() > 0){
//                destGen (numdst->intValue());
//            }
//            int numParcel = 500;
//            parcelsDefinition(numParcel);
///*           for (auto i:parcel_depot){
//               std::cout << i.parcelID <<" ; W =" <<i.weight <<"; P = " << i.priority << "; Exp = " <<i.exp_time
//                       <<" Pos = (" <<i.parceldest.x <<"; " << i.parceldest.y <<"; " <<i.parceldest.z <<")" << std::endl;
//           }*/
//        }
///*        destination = missionPathNextDest(originPos);
//        std::cout << " Name -----> " << getParentModule()->getFullName() <<
//                    " And Speed = " << speedParameter->doubleValue() << std::endl;*/
//    }
//}
//
//void DroneNetMob::orient()
//{
//    if (faceForward)
//        lastOrientation = quaternion;
//}
//
//void DroneNetMob::setTargetPosition()
//{
//    if (flag_original){
//        rad angleDelta = deg(angleDeltaParameter->doubleValue());
//        rad rotationAxisAngle = deg(rotationAxisAngleParameter->doubleValue());
//        Quaternion dQ = Quaternion(Coord::X_AXIS, rotationAxisAngle.get()) * Quaternion(Coord::Z_AXIS, angleDelta.get());
//        quaternion = quaternion * dQ;
//        quaternion.normalize();
//        Coord direction = quaternion.rotate(Coord::X_AXIS);
//
//        simtime_t nextChangeInterval = *changeIntervalParameter;
//        EV_DEBUG << "interval: " << nextChangeInterval << endl;
//        sourcePosition = lastPosition;
//        targetPosition = lastPosition + direction * (*speedParameter) * nextChangeInterval.dbl();
//        previousChange = simTime();
//        nextChange = previousChange + nextChangeInterval;
//    }
//    else{
//        simtime_t nextChangeInterval = *changeIntervalParameter;
//        sourcePosition = lastPosition;
////            destination = missionPathNextDest(originPos);
//        targetPosition = missionPathNextDest(lastPosition);
//        previousChange = simTime();
//        nextChange = previousChange + nextChangeInterval;
//    }
//}
//
//void DroneNetMob::move()
//{
//    if (flag_original){
//        simtime_t now = simTime();
//        rad dummyAngle;
//        if (now == nextChange) {
//            lastPosition = targetPosition;
//            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
//            EV_INFO << "reached current target position = " << lastPosition << endl;
//            setTargetPosition();
//            EV_INFO << "new target position = " << targetPosition << ", next change = " << nextChange << endl;
//            lastVelocity = (targetPosition - lastPosition) / (nextChange - simTime()).dbl();
//            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
//        }
//        else if (now > lastUpdate) {
//            ASSERT(nextChange == -1 || now < nextChange);
//            double alpha = (now - previousChange) / (nextChange - previousChange);
//            lastPosition = sourcePosition * (1 - alpha) + targetPosition * alpha;
//            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
//        }
//    }
//    else{
//        simtime_t now = simTime();
//        rad dummyAngle;
//        if (now == nextChange) {
//            lastPosition = targetPosition;
//            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
//            EV_INFO << "reached current target position = " << lastPosition << endl;
//            setTargetPosition();
//            EV_INFO << "new target position = " << targetPosition << ", next change = " << nextChange << endl;
//            lastVelocity = (targetPosition - lastPosition) / (nextChange - simTime()).dbl();
//            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
//            std::cout <<"Vel = " << lastVelocity.x <<"  ; " << lastVelocity.x << "  ; " << lastVelocity.z << std::endl;
//        }
//        else if (now > lastUpdate) {
//            ASSERT(nextChange == -1 || now < nextChange);
//            double alpha = (now - previousChange) / (nextChange - previousChange);
//            lastPosition = sourcePosition * (1 - alpha) + targetPosition * alpha;
//            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
//
//            if (std::strcmp(getParentModule()->getFullName(), "drone[1]") == 0){
//                std::cout <<"Vel = " << lastVelocity.x <<"  ; " << lastVelocity.x << "  ; " << lastVelocity.z << std::endl;
//            }
//
///*           std::cout << " Name -----> " << getParentModule()->getFullName() << " , Velocity = ("
//                            << getCurrentVelocity().x << "; " << getCurrentVelocity().y << "; "<< getCurrentVelocity().z <<"), Position = ("
//                            << getCurrentPosition().x<< "; " << getCurrentPosition().y <<"; "<< getCurrentPosition().z <<")"<< std::endl;*/
//        }
//    }
//}
//
//double DroneNetMob::getMaxSpeed() const
//{
//    return speedParameter->isExpression() ? NaN : speedParameter->doubleValue();
//}
//
//void DroneNetMob::destGen(int ndst){
//    for (unsigned int i = 0; i < numdst->intValue(); i++){
//        Coord nextdst;
//        nextdst.x = rand() % 600;
//        nextdst.y = rand() % 400;
//        nextdst.z = 0;
//        dst.push_back(nextdst);
//    }
///*    std::cout <<std::endl <<std::endl;
//    for(auto i:dst){
//        std::cout <<"(" <<i.x <<"; " << i.y <<"; " <<i.z <<") | ";
//    }
//    std::cout <<std::endl <<std::endl;*/
//}
//void DroneNetMob::parcelsDefinition (int nparcels){
//    for (unsigned int i = 0; i < nparcels; i++){
//        parcel tmpparcel;
//        parcel *p;
//        tmpparcel.parcelID = i;
//        tmpparcel.weight =  rand() % 10 + 1;
//        tmpparcel.priority = 1;
//        tmpparcel.exp_time = rand() % 300;
//        int n = numdst->intValue();
//        int dindex = rand() % n;
//        tmpparcel.parceldest = dst[dindex];
//        parcel_depot.push_back(tmpparcel);
//
//    }
//}
//std::vector<parcel> DroneNetMob::droneParcelsSelectionFromSource(int parcelSel){
//    std::vector<parcel> selectedParcels;
//    double packedweight = 0;
//    cout <<" build is done"  << endl;
//    std::cout << " Selection ===>  " << parcelSel << std::endl;
//    std::cout << " Drone -----> " << getParentModule()->getFullName() <<
//                    " with speed = " << speedParameter->doubleValue() <<" Defines its mission!"<< std::endl;
//    switch(parcelSel){
//        /* Closest-Deadline-Parcel-First
//         * Depot sorted by Deadline
//         * First deliver the ones with small deadline*/
//        case CDPF:{
//            std::cout <<" CDPF ----- >    Parcel Selection Method! parcel_depot size = " <<parcel_depot.size() << std::endl;
//            if (!flagArrangedDepot){
//                std::sort (parcel_depot.begin(), parcel_depot.end(),sortDepotByDeadline);
///*                for (unsigned int i = 0; i < parcel_depot.size(); i++){
//                    std::cout <<"P" <<parcel_depot[i].parcelID << ":: " << "W = " <<parcel_depot[i].weight <<" dst = (" <<parcel_depot[i].parceldest.x <<"; "<<parcel_depot[i].parceldest.y <<"; "
//                            <<parcel_depot[i].parceldest.z <<")"<<" Deadline = "<< parcel_depot[i].exp_time  <<std::endl;
//                }*/
//                int k=0;
//                for (unsigned int i = 0; i < parcel_depot.size(); i++){
//                    packedweight+=parcel_depot[i].weight;
//                    if (packedweight < droneweightcapacity){
//                        selectedParcels.push_back(parcel_depot[i]);
//                        k++;
//                    }
//                    else{
//                        break;
//                    }
//                }
//                std::cout <<"11111 Number of removed Parcels = " <<k<< "|| Depot = " <<parcel_depot.size() <<std::endl;
//                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
//                std::cout <<"11111 Number of removed Parcels = " <<k<< "|| Depot = " <<parcel_depot.size() <<std::endl;
//            }
//            else{
//                int k=0;
//                for (unsigned int i = 0; i < parcel_depot.size(); i++){
//                    packedweight+=parcel_depot[i].weight;
//                    if (packedweight < droneweightcapacity){
//                        selectedParcels.push_back(parcel_depot[i]);
//                        k++;
//                    }
//                    else{
//                        break;
//                    }
//                }
//                std::cout <<"22222 Number of removed Parcels = " <<k<<" || Depot = " <<parcel_depot.size()<<std::endl;
//                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
//                std::cout <<"22222 Number of removed Parcels = " <<k<<" || Depot = " <<parcel_depot.size()<<std::endl;
//            }
//            break;
//        }
//        /* Closest-Neighbor-Parcel-First
//         * Depot sorted by Positions
//         * First deliver the ones closer to source*/
//        case CNPF:{
//            cout <<" CNPF ----- > working~~~~~~~~~~~~~"  << endl;
//            if (!flagArrangedDepot){
//                std::sort (parcel_depot.begin(), parcel_depot.end(),sortDepotByDestination);
//                int k=0;
//                for (unsigned int i = 0; i < parcel_depot.size(); i++){
//                    packedweight+=parcel_depot[i].weight;
//                    if (packedweight < droneweightcapacity){
//                        selectedParcels.push_back(parcel_depot[i]);
//                        k++;
//                    }
//                    else{
//                        break;
//                    }
//                }
//                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
//            }
//            else{
//                int k=0;
//                for (unsigned int i = 0; i < parcel_depot.size(); i++){
//                    packedweight+=parcel_depot[i].weight;
//                    if (packedweight < droneweightcapacity){
//                        selectedParcels.push_back(parcel_depot[i]);
//                        k++;
//                    }
//                    else{
//                        break;
//                    }
//                }
//                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
//            }
//            break;
//        }
//        /* Efficient Parcel Delivery Service
//         * Depot sorted in a greedy way ()
//         * First deliver the ones with small ratio distance/weight*/
//        case EPDS:{
//            if (!flagArrangedDepot){
//                std::sort (parcel_depot.begin(), parcel_depot.end(),greedySortDepot);
//                int k=0;
//                for (unsigned int i = 0; i < parcel_depot.size(); i++){
//                    packedweight+=parcel_depot[i].weight;
//                    if (packedweight < droneweightcapacity){
//                        selectedParcels.push_back(parcel_depot[i]);
//                        k++;
//                    }
//                    else{
//                        break;
//                    }
//                }
//                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
//            }
//            else{
//                int k=0;
//                for (unsigned int i = 0; i < parcel_depot.size(); i++){
//                    packedweight+=parcel_depot[i].weight;
//                    if (packedweight < droneweightcapacity){
//                        selectedParcels.push_back(parcel_depot[i]);
//                        k++;
//                    }
//                    else{
//                        break;
//                    }
//                }
//                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
//            }
//            break;
//        }
//        /* Randomly-Selected-Parcel-First
//         * Randomly select Parcels to be delivered first*/
//        case RSPF:{
//            if (!flagArrangedDepot){
//
//            }
//            else{
//
//            }
//            break;
//        }
//        /* Heaviest Parcel First
//         * Depot is sorted based on parcel weight
//         * First deliver the heaviest ones to the lightest*/
//        case HPF:{
//            if (!flagArrangedDepot){
//                std::sort (parcel_depot.begin(), parcel_depot.end(),SortDepotByWeight);
//                int k=0;
//                for (unsigned int i = 0; i < parcel_depot.size(); i++){
//                    packedweight+=parcel_depot[i].weight;
//                    if (packedweight < droneweightcapacity){
//                        selectedParcels.push_back(parcel_depot[i]);
//                        k++;
//                    }
//                    else{
//                        break;
//                    }
//                }
//                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
//            }
//            else{
//                int k=0;
//                for (unsigned int i = 0; i < parcel_depot.size(); i++){
//                    packedweight+=parcel_depot[i].weight;
//                    if (packedweight < droneweightcapacity){
//                        selectedParcels.push_back(parcel_depot[i]);
//                        k++;
//                    }
//                    else{
//                        break;
//                    }
//                }
//                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
//            }
//            break;
//        }
//        default:{
//            std::cout <<" Undefined Selection Method." <<std::endl;
//        }
//    }
//    for (auto i:selectedParcels){
//        std::cout <<"+++ ID = " << i.parcelID << " Weight: " <<i.weight <<" deadline = " << i.exp_time <<
//                "   <<-->> parcel_depot Size = "<< parcel_depot.size() <<std::endl;
//    }
//    return selectedParcels;
//}
//Coord DroneNetMob::missionPathNextDest(Coord cpos){
//    Coord nextdest;
//    if (!OngoingMission){
//        MissionParcels  = droneParcelsSelectionFromSource(selectionMethod);
//        OngoingMission = true;
//    }
//    else{
//        if (MissionParcels.size() == 0){
//            nextdest = originPos;
//            OngoingMission = false;
//        }
//        else{
//            double nearestDist = 0;
//            int k = 0; //Next Parcel Index
//            for (unsigned int i = 0; i < MissionParcels.size(); i++){
//                if (i == 0){
//                    double tmpd = sqrt(pow(MissionParcels[i].parceldest.x - cpos.x, 2)
//                                        + pow(MissionParcels[i].parceldest.y - cpos.y, 2)
//                                        +pow(MissionParcels[i].parceldest.z - cpos.z, 2));
//                    nextdest = MissionParcels[i].parceldest;
//                    nearestDist = tmpd;
//                    k = i;
//                }
//                else{
//                    double tmpd = sqrt(pow(MissionParcels[i].parceldest.x - cpos.x, 2)
//                                        + pow(MissionParcels[i].parceldest.y - cpos.y, 2)
//                                        +pow(MissionParcels[i].parceldest.z - cpos.z, 2));
//                    if (tmpd < nearestDist){
//                        nextdest = MissionParcels[i].parceldest;
//                        nearestDist = tmpd;
//                        k = i;
//                    }
//                }
//            }
//            MissionParcels.erase(MissionParcels.begin()+k);
//        }
//    }
//    std::cout <<" Destination --- > (" <<nextdest.x <<"; " <<nextdest.y <<"; " <<nextdest.z << ")"
//            <<" Origin ---> (" <<originPos.x <<"; "<<originPos.y << "; " << originPos.z<< ")" << std::endl;
//    return nextdest;
//}
//
///*Algorithm for destination selection based on:
// * Shortest path and drone capacity*/
//Coord DroneNetMob::destAssignment(){
//    int sz = dst.size();
//    Coord ds = dst[gen];
//    gen++;
//    return ds;
//}
//
//
//
//} // namespace inet
//
