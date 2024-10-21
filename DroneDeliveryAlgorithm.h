#ifndef DRONEDELIVERYALGORITHMS_H
#define DRONEDELIVERYALGORITHMS_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>
#include "inet/common/INETDefs.h"
#include "inet/mobility/base/LineSegmentsMobilityBase.h"

using namespace inet;

struct parcel {
    int parcelID;
    double weight;
    int priority;
    double exp_time;
    Coord parceldest;
};

class DroneDeliveryAlgorithms {
public:
    static double dist(const Coord& a, const Coord& b);
    static double batteryCalculation(const Coord& a, const Coord& b, double carriedWeight, double speed);
    // static std::vector<parcel> energyEfficientPath(std::vector<parcel>& parcels, double& totalDistance);
    // static void remove_dupcoordinates(std::vector<parcel>& parcels, int& carriedParcels, int& totalparcel);
    // static std::vector<parcel> greedyTSP(std::vector<parcel>& parcels, double& distance);
    // static std::vector<parcel> greedyTSP_B(std::vector<parcel>& parcels, double& distance, double carriedWeight, double speed);
    // static std::vector<parcel> dfs_bnb(std::vector<parcel>& parcels, double& distance, int& carriedParcels);
    // static std::vector<parcel> dp_tsp(std::vector<parcel>& parcels, double& totaldistance, int& carriedParcels, int& totalparcel);
    // static std::vector<parcel> mstTSP(std::vector<parcel>& parcels);

private:
    // static double calculateMST(const std::vector<std::vector<double>>& distances, const std::vector<bool>& included);
    // static double calculate1TreeLowerBound(const std::vector<std::vector<double>>& travel, int start, const std::vector<bool>& visited);
    // static void dfs(int start, int next, double value, std::vector<int>& visited, int n, std::vector<std::vector<double>>& travel, double& min_value, std::vector<int>& path);
    // static double dynamic(std::vector<parcel>& parcels, int pos, int visited, std::vector<std::vector<double>>& dp, std::vector<std::vector<double>>& distance, double currDist, double& ans, std::vector<int>& optimalPath, std::vector<int>& path);
};

#endif // DRONEDELIVERYALGORITHMS_H