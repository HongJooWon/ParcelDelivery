#include "Dest.h"
#include "inet/common/INETMath.h"

namespace inet{

Define_Module(Dest);

Dest::Dest(){

}

Dest::~Dest(){

}

void Dest::initialize(){
    droneArrival = new cMessage("Arrival of Drone");
}
void Dest::handleMessage(cMessage* msg){
    if (msg == droneArrival){
    }
    else{
    }
}

}
