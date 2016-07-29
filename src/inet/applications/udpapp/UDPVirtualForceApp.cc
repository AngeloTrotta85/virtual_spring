//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 


#include <applications/udpapp/UDPVirtualForceApp.h>

#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/NodeOperations.h"
#include "inet/transportlayer/contract/udp/UDPControlInfo_m.h"

namespace inet {

Define_Module(UDPVirtualForceApp);

UDPVirtualForceApp::~UDPVirtualForceApp() {
    cancelAndDelete(autoMsg);
}

void UDPVirtualForceApp::initialize(int stage)
{
    UDPBasicApp::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        autoMsg = new cMessage("updateForce");
        scheduleAt(simTime() + dblrand(), autoMsg);

        mob = dynamic_cast<VirtualSpringMobility *>(this->getParentModule()->getSubmodule("mobility"));
    }
}

void UDPVirtualForceApp::finish() {
    UDPBasicApp::finish();
}

void UDPVirtualForceApp::handleMessage(cMessage *msg)
{
    if ((msg->isSelfMessage()) && (msg == autoMsg)) {
        updateForce();
        scheduleAt(simTime() + 0.1, autoMsg);
    }
    else {
        UDPBasicApp::handleMessage(msg);
    }
}

void UDPVirtualForceApp::updateForce(void) {
    int numberNodes = this->getParentModule()->getVectorSize();
    Coord myPos = mob->getCurrentPosition();

    // clear everything
    mob->clearVirtualSprings();

    for (int i = 0; i < numberNodes; i++) {
        if (i != this->getParentModule()->getIndex()) {
            VirtualSpringMobility *mobNeigh = dynamic_cast<VirtualSpringMobility *>(this->getParentModule()->getParentModule()->getSubmodule("host", i)->getSubmodule("mobility"));
            Coord neighPos = mobNeigh->getCurrentPosition();

            double distance = neighPos.distance(myPos);

            if (distance < 200.0) {
                double springDispl = 100.0 - distance;

                Coord uVec = neighPos - myPos;
                uVec.normalize();

                EV << "Setting force with displacement: " << springDispl << " (distance: " << distance << ")" << endl;

                mob->addVirtualSpring(uVec, 100.0, springDispl);
            }
        }
    }
}


} /* namespace inet */
