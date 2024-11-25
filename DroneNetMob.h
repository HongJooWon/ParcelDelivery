/*
 * DroneNetMob.h
 *
 *  Created on: June 14, 2021
 *      Author: iotlab_aime
 */

#ifndef __INET_DRONENETMOB_H
#define __INET_DRONENETMOB_H

#include "inet/common/INETDefs.h"
#include "inet/mobility/base/LineSegmentsMobilityBase.h"
#include <vector>
#include <algorithm>
#include <iostream>
//----------------------------------------------
#ifdef _WIN32
#ifdef INET_EXPORTS
#define INET_API __declspec(dllexport)
#else
#define INET_API __declspec(dllimport)
#endif
#else
#define INET_API
#endif

//--------------------------------------------------

using namespace std;
namespace inet {

/**
 * @brief Random mobility model for a mobile host with a mass.
 * See NED file for more info.
 *
 * @author Emin Ilker Cetinbas, Andras Varga
 */
struct parcel{
    int parcelID;
    double weight;
    int priority;
    double exp_time;
    Coord parceldest;
};

struct ParcelScore {
    parcel p;
    double score;
};

struct GroupedParcel {
    vector<parcel> parcels;  // 같은 목적지의 택배들
    double totalWeight;      // 해당 목적지 택배들의 총 무게
    Coord dest;             // 공통 목적지
    
    GroupedParcel() : totalWeight(0) {}
};

class INET_API DroneNetMob : public LineSegmentsMobilityBase
{
  protected:
    // config (see NED file for explanation)
    cPar *changeIntervalParameter = nullptr;
    cPar *angleDeltaParameter = nullptr;
    cPar *rotationAxisAngleParameter = nullptr;
    cPar *speedParameter = nullptr;
    cPar *numdst = nullptr;
    cPar *ox = nullptr;
    cPar *oy = nullptr;
    cPar *oz = nullptr;

    //delivery info
    bool flag_original = false;
    vector<Coord> dst; //Destination Positions
    int gen = 0;
    bool flagArrangedDepot = false;
    bool OngoingMission = false;
    bool isEnd = false;
    vector<parcel> parcel_depot;
    vector<parcel> shared_parcel_depot;  // 추가: 공유용 택배 리스트

    // state
    Quaternion quaternion;
    simtime_t previousChange;
    Coord sourcePosition;
    Coord destination; //BAM
    // bool flagmovedtodst;
    // double droneweightcapacity;
    // double droneremainingbattery;
    // double totalWeight;
    // int selectionMethod;
    // std::vector<parcel> MissionParcels;
    // double deliveryStartTime = 0;
    // double deliveryEndTime =   0;
    // double batteryConsumption = 0;
    // double expectedBatteryConsumption = 0;
    // double tspDistance = 0;

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }

    /** @brief Initializes mobility model parameters. */
    virtual void initialize(int stage) override;

    /** @brief Move the host according to the current simulation time. */
    virtual void move() override;
    void orient() override;

    /** @brief Calculate a new target position to move to. */
    virtual void setTargetPosition() override;
//    virtual void handleMessage(cMessage *msg) override;


  public:
    DroneNetMob();
    virtual double getMaxSpeed() const override;
    void destGen(int ndst);
    void parcelsDefinition (int nparcels);
    // std::vector<parcel> efficientTrajectoryDesign(std::vector<parcel>& parcels, double& totalDistance);
    // std::vector<parcel> efficientTrajectoryDesign();    
    // void parcelsDefinition (std::string filename);
    std::vector<parcel> droneParcelsSelectionFromSource(int parcelSel);
    Coord missionPathNextDest(Coord curpos);
    Coord destAssignment();
    //std::vector<parcel> greedyTSP_B();
    double droneWeightCapacity;
    double droneremainingbattery;
    double carriedWeight;
    int selectionMethod;
    std::vector<parcel> MissionParcels;
    std::vector<parcel> sortedParcels;
    double deliveryStartTime = 0;
    double deliveryEndTime =   0;
    double batteryConsumption = 0;
    double expectedBatteryConsumption = 0;
    double tspDistance = 0;
  
  private:
    simtime_t missionTime = 0;
    int totalParcels = -1;
    int carriedParcels = 0;
};

} // namespace inet

#endif // ifndef __INET_DRONENETMOB_H

