#include "DroneDeliveryAlgorithm.h"
#include <iostream>

double DroneDeliveryAlgorithms::dist(const Coord& a, const Coord& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

double DroneDeliveryAlgorithms::batteryCalculation(const Coord& a, const Coord& b, double carriedWeight, double speed) {
    double distance = dist(a, b);
    double batteryConsumption = 0;
    double time = distance / speed;
    if (carriedWeight == 0) {
        batteryConsumption = 308.709 * time - 0.852;
    } else {
        batteryConsumption = (0.311 * carriedWeight + 1.735) * time;
    }
    return batteryConsumption;
}

//Greedy - Heaviest Parcel First

//Greedy - Nearst Parcel First

//Greedy - Least Energy Consumption First