//
// Copyright (C) 2006 Andras Varga
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#include "StatNodeMob.h"

namespace inet {
std::map<std::string, Coord> DstNodesPos;
std::map<std::string, Coord> BstNodesPos;

Define_Module(StatNodeMob);

void StatNodeMob::initialize(int stage)
{
    StationaryMobility::initialize(stage);
    if (stage == INITSTAGE_LOCAL)
        updateFromDisplayString = par("updateFromDisplayString");
//    std::cout <<"111111111111111111111111111" <<std::endl;
    if (strcmp(getParentModule()->getName(), "dst") == 0){
        std::cout << " Name -----> " << getParentModule()->getFullName() << " stationaryNode.cc" <<std::endl;
        std::map<std::string, Coord>::iterator it = DstNodesPos.find(getParentModule()->getFullName());
        if (it == DstNodesPos.end()){
            Coord InPos;
            InPos.x = par("initialX").doubleValue();
            InPos.y = par("initialY").doubleValue();
            InPos.z = par("initialZ").doubleValue();

//            DstNodesPos.push_back(InPos);
            DstNodesPos.insert(std::make_pair(getParentModule()->getFullName(), InPos));
            std::cout <<" Dest: " <<getParentModule()->getFullName()<<"(" <<InPos.x <<"; " <<InPos.y <<"; " <<InPos.z <<")" <<std::endl;
        }
    }
    else if (strcmp(getParentModule()->getName(), "bstion") == 0){
        std::map<std::string, Coord>::iterator it = BstNodesPos.find(getParentModule()->getFullName());
        if (it == BstNodesPos.end()){
            Coord InPos;
            InPos.x = par("initialX").doubleValue();
            InPos.y = par("initialY").doubleValue();
            InPos.z = par("initialZ").doubleValue();

            BstNodesPos.insert(std::make_pair(getParentModule()->getFullName(), InPos));
//            std::cout <<" Src: " <<getParentModule()->getFullName()<<"(" <<InPos.x <<"; " <<InPos.y <<"; " <<InPos.z <<")" <<std::endl;
        }
    }
}

void StatNodeMob::refreshDisplay() const
{
    if (updateFromDisplayString) {
        const_cast<StatNodeMob *>(this)->updateMobilityStateFromDisplayString();
        DirectiveResolver directiveResolver(const_cast<StatNodeMob *>(this));
        auto text = format.formatString(&directiveResolver);
//240418 delete for remove error
//         getDisplayString().setTagArg("t", 0, text);

    }
//    else
//        StationaryMobility::refreshDisplay();
}

void StatNodeMob::updateMobilityStateFromDisplayString()
{
    char *end;
    double depth;
    cDisplayString& displayString = subjectModule->getDisplayString();
    canvasProjection->computeCanvasPoint(lastPosition, depth);
    double x = strtod(displayString.getTagArg("p", 0), &end);
    double y = strtod(displayString.getTagArg("p", 1), &end);
    auto newPosition = canvasProjection->computeCanvasPointInverse(cFigure::Point(x, y), depth);
    if (lastPosition != newPosition) {
        lastPosition = newPosition;
        emit(mobilityStateChangedSignal, const_cast<StatNodeMob *>(this));
    }
    Quaternion swing;
    Quaternion twist;
    Coord vector = canvasProjection->computeCanvasPointInverse(cFigure::Point(0, 0), 1);
    vector.normalize();
//    lastOrientation.getSwingAndTwist(vector, swing, twist);
    double oldAngle;
    Coord axis;
    twist.getRotationAxisAndAngle(axis, oldAngle);
    double newAngle = math::deg2rad(strtod(displayString.getTagArg("a", 0), &end));
    if (oldAngle != newAngle) {
        lastOrientation *= Quaternion(vector, newAngle - oldAngle);
        emit(mobilityStateChangedSignal, const_cast<StatNodeMob *>(this));
    }
}
std::map<std::string, Coord> StatNodeMob::returnDestinations(){
    std::cout <<" DstNodesPos Size = " << DstNodesPos.size() <<std::endl;
    return DstNodesPos;
}
std::map <std::string, Coord> StatNodeMob::returnBStations(){
    return BstNodesPos;
}

} // namespace inet

