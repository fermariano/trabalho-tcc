//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#pragma once

#include <fstream>
#include <sstream>
#include <string>

#include "veins/veins.h"

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

using namespace omnetpp;

namespace veins {

/**
 * @brief
 * This is a stub for a typical Veins application layer.
 * Most common functions are overloaded.
 * See MyVeinsApp.cc for hints
 *
 * @author David Eckhoff
 *
 */

class VEINS_API MyVeinsApp : public DemoBaseApplLayer {
public:
    ~MyVeinsApp() override;
    void initialize(int stage) override;
    void finish() override;

protected:
    enum SelfMsgKinds {
        LEADER_BEACON_EVT = 2000,
        LEADER_BRAKE_EVT = 2001,
        APPLY_LEADER_CMD_EVT = 2002,
    };

    void onBSM(DemoSafetyMessage* bsm) override;
    void onWSM(BaseFrame1609_4* wsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;

    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;

    simtime_t samplePqcDelay(simtime_t baseDelay) const;
    int resolveVehicleIndex() const;
    void sendLeaderBeacon();
    void scheduleFollowerCommand(const DemoSafetyMessage* bsm);
    void applyFollowerCommand(cMessage* msg);
    void cleanupTimers();

protected:
    bool isLeader = false;
    int vehicleIndex = -1;
    int leaderVehicleId = 0;

    int platoonBeaconPsid = 77;
    double platoonTargetGap = 15;
    double platoonCruiseSpeed = 25;
    double followerControllerGain = 0.35;
    simtime_t followerControlHorizon = SIMTIME_ZERO;
    simtime_t leaderBeaconInterval = SIMTIME_ZERO;
    simtime_t leaderBrakeTime = SIMTIME_ZERO;
    simtime_t leaderBrakeDuration = SIMTIME_ZERO;
    double leaderBrakeTargetSpeed = 0;

    int pqcSignatureBytes = 2420;
    int pqcPublicKeyBytes = 1312;
    simtime_t pqcSignDelay = SIMTIME_ZERO;
    simtime_t pqcVerifyDelay = SIMTIME_ZERO;
    simtime_t pqcDelayJitter = SIMTIME_ZERO;
    bool randomizePqcDelay = false;

    cMessage* leaderBeaconTimer = nullptr;
    cMessage* leaderBrakeTimer = nullptr;

    simsignal_t pqcPayloadBitsSignal;
    simsignal_t pqcSignDelaySignal;
    simsignal_t pqcVerifyDelaySignal;
    simsignal_t appliedSpeedSignal;

    // Fragmentation + transmission metrics
    int mac80211pMtuBytes;
    int mac80211pBitrateKbps;
    simsignal_t fragmentCountSignal;
    simsignal_t txDelaySignal;
    simsignal_t brakeLatencySignal;

    // Brake latency tracking
    simtime_t leaderBrakeActualTime;  // set by leader when brake fires

    // File-based logging
    std::string logFilePrefix;
    std::ofstream logFile;
    void writeLog(const std::string& line);
};

} // namespace veins
