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

    // state
    Quaternion quaternion;
    simtime_t previousChange;
    Coord sourcePosition;
    Coord destination; //BAM
//    bool flagmovedtodst;
    double droneweightcapacity;
    double droneremainingbattery;
    int selectionMethod;
    std::vector<parcel> MissionParcels;
    double deliveryStartTime = 0;
    double deliveryEndTime =   0;

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
    std::vector<parcel> droneParcelsSelectionFromSource(int parcelSel);
    Coord missionPathNextDest(Coord curpos);
    Coord destAssignment();
  
  private:
    double tspDistance = 0;
    simtime_t missionTime = 0;
    int totalParcels = -1;
    int carriedParcels = 0;
};

} // namespace inet

#endif // ifndef __INET_DRONENETMOB_H

