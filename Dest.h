#include "inet/common/INETDefs.h"
//#include "inet/src/inet/common/INETDefs.h"
#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <map>

namespace inet{

using namespace std;
using namespace omnetpp;
class Dest : public cSimpleModule{
    private:
        virtual void initialize();
        virtual void handleMessage(cMessage *msg);
        cMessage* droneArrival;

    public:
        Dest();
        ~Dest();
};

}
