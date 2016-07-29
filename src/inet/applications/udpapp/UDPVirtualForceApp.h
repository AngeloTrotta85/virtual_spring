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

#ifndef INET_APPLICATIONS_UDPAPP_UDPVIRTUALFORCEAPP_H_
#define INET_APPLICATIONS_UDPAPP_UDPVIRTUALFORCEAPP_H_

#include <vector>

#include "inet/common/INETDefs.h"

#include "inet/applications/udpapp/UDPBasicApp.h"
#include "inet/transportlayer/contract/udp/UDPSocket.h"

#include "inet/mobility/single/VirtualSpringMobility.h"

namespace inet {

class UDPVirtualForceApp : public UDPBasicApp {

protected:
    cMessage *autoMsg = nullptr;

    VirtualSpringMobility *mob = nullptr;

protected:
  virtual int numInitStages() const override { return NUM_INIT_STAGES; }
  virtual void initialize(int stage) override;
  virtual void handleMessage(cMessage *msg) override;
  virtual void finish() override;

private:
  void updateForce(void);

public:
    UDPVirtualForceApp() {};
    virtual ~UDPVirtualForceApp();
};

} /* namespace inet */

#endif /* INET_APPLICATIONS_UDPAPP_UDPVIRTUALFORCEAPP_H_ */
