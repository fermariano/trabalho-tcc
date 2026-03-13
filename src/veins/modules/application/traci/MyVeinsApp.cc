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

#include "veins/modules/application/traci/MyVeinsApp.h"

#include <algorithm>
#include <exception>

using namespace veins;

Define_Module(veins::MyVeinsApp);

MyVeinsApp::~MyVeinsApp()
{
    cleanupTimers();
}

void MyVeinsApp::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        isLeader = false;
        vehicleIndex = -1;

        leaderVehicleId = par("platoonLeaderId").intValue();
        platoonBeaconPsid = par("platoonBeaconPsid").intValue();
        platoonTargetGap = par("platoonTargetGap").doubleValue();
        platoonCruiseSpeed = par("platoonCruiseSpeed").doubleValue();
        followerControllerGain = par("followerControllerGain").doubleValue();
        followerControlHorizon = par("followerControlHorizon");
        leaderBeaconInterval = par("leaderBeaconInterval");
        leaderBrakeTime = par("leaderBrakeTime");
        leaderBrakeDuration = par("leaderBrakeDuration");
        leaderBrakeTargetSpeed = par("leaderBrakeTargetSpeed").doubleValue();

        pqcSignatureBytes = par("pqcSignatureBytes").intValue();
        pqcPublicKeyBytes = par("pqcPublicKeyBytes").intValue();
        pqcSignDelay = par("pqcSignDelay");
        pqcVerifyDelay = par("pqcVerifyDelay");
        pqcDelayJitter = par("pqcDelayJitter");
        randomizePqcDelay = par("randomizePqcDelay").boolValue();

        leaderBeaconTimer = nullptr;
        leaderBrakeTimer = nullptr;

        pqcPayloadBitsSignal = registerSignal("pqcPayloadBits");
        pqcSignDelaySignal = registerSignal("pqcSignDelay");
        pqcVerifyDelaySignal = registerSignal("pqcVerifyDelay");
        appliedSpeedSignal = registerSignal("appliedSpeed");
    }
    else if (stage == 1) {
        vehicleIndex = resolveVehicleIndex();
        isLeader = (vehicleIndex == leaderVehicleId);

        if (isLeader) {
            leaderBeaconTimer = new cMessage("leaderBeaconTimer", LEADER_BEACON_EVT);
            leaderBrakeTimer = new cMessage("leaderBrakeTimer", LEADER_BRAKE_EVT);
            scheduleAt(simTime() + leaderBeaconInterval, leaderBeaconTimer);
            scheduleAt(leaderBrakeTime, leaderBrakeTimer);
            if (traciVehicle) traciVehicle->setSpeed(platoonCruiseSpeed);
        }

        EV_INFO << "Platoon app configured for vehicleId=" << vehicleIndex << " leader=" << isLeader << std::endl;
    }
}

void MyVeinsApp::finish()
{
    cleanupTimers();
    DemoBaseApplLayer::finish();

    const long pqcOverheadBits = (static_cast<long>(std::max(0, pqcSignatureBytes)) + static_cast<long>(std::max(0, pqcPublicKeyBytes))) * 8L;
    recordScalar("vehicleIndex", vehicleIndex);
    recordScalar("isLeader", isLeader ? 1 : 0);
    recordScalar("pqcOverheadBitsPerBeacon", pqcOverheadBits);
}

void MyVeinsApp::onBSM(DemoSafetyMessage* bsm)
{
    if (isLeader) return;
    if (bsm->getPsid() != platoonBeaconPsid) return;
    scheduleFollowerCommand(bsm);
}

void MyVeinsApp::onWSM(BaseFrame1609_4* wsm)
{
    EV_DEBUG << "Ignoring WSM " << wsm->getName() << std::endl;
}

void MyVeinsApp::onWSA(DemoServiceAdvertisment* wsa)
{
    EV_DEBUG << "Ignoring WSA " << wsa->getName() << std::endl;
}

void MyVeinsApp::handleSelfMsg(cMessage* msg)
{
    if (msg == leaderBeaconTimer) {
        sendLeaderBeacon();
        scheduleAt(simTime() + leaderBeaconInterval, leaderBeaconTimer);
        return;
    }

    if (msg == leaderBrakeTimer) {
        if (traciVehicle) {
            EV_INFO << "Leader braking at " << simTime() << " from " << platoonCruiseSpeed << "m/s to " << leaderBrakeTargetSpeed << "m/s over " << leaderBrakeDuration << std::endl;
            traciVehicle->slowDown(leaderBrakeTargetSpeed, leaderBrakeDuration);
        }
        return;
    }

    if (msg->getKind() == APPLY_LEADER_CMD_EVT) {
        applyFollowerCommand(msg);
        return;
    }

    DemoBaseApplLayer::handleSelfMsg(msg);
}

void MyVeinsApp::handlePositionUpdate(cObject* obj)
{
    DemoBaseApplLayer::handlePositionUpdate(obj);
}

simtime_t MyVeinsApp::samplePqcDelay(simtime_t baseDelay) const
{
    if (!randomizePqcDelay || pqcDelayJitter <= SIMTIME_ZERO) return baseDelay;
    return baseDelay + SimTime(uniform(0, pqcDelayJitter.dbl()));
}

int MyVeinsApp::resolveVehicleIndex() const
{
    if (!mobility) return getParentModule()->getIndex();

    const std::string externalId = mobility->getExternalId();
    try {
        size_t used = 0;
        const int parsed = std::stoi(externalId, &used);
        if (used == externalId.size()) return parsed;
    }
    catch (const std::exception&) {
    }

    EV_WARN << "Could not parse numeric SUMO id '" << externalId << "'. Falling back to module index." << std::endl;
    return getParentModule()->getIndex();
}

void MyVeinsApp::sendLeaderBeacon()
{
    if (!isLeader) return;

    DemoSafetyMessage* bsm = new DemoSafetyMessage("platoon-bsm");
    populateWSM(bsm);
    bsm->setPsid(platoonBeaconPsid);

    const int extraBytes = std::max(0, pqcSignatureBytes) + std::max(0, pqcPublicKeyBytes);
    bsm->addBitLength(extraBytes * 8);

    const simtime_t signDelay = samplePqcDelay(pqcSignDelay);
    emit(pqcPayloadBitsSignal, static_cast<long>(bsm->getBitLength()));
    emit(pqcSignDelaySignal, signDelay.dbl());

    if (signDelay > SIMTIME_ZERO) {
        sendDelayedDown(bsm, signDelay);
    }
    else {
        sendDown(bsm);
    }
}

void MyVeinsApp::scheduleFollowerCommand(const DemoSafetyMessage* bsm)
{
    if (!traciVehicle) return;

    cMessage* cmd = new cMessage("applyLeaderCommand", APPLY_LEADER_CMD_EVT);
    cmd->addPar("leaderSpeed") = bsm->getSenderSpeed().length();
    cmd->addPar("distanceToLeader") = curPosition.distance(bsm->getSenderPos());

    const simtime_t verifyDelay = samplePqcDelay(pqcVerifyDelay);
    emit(pqcVerifyDelaySignal, verifyDelay.dbl());
    scheduleAt(simTime() + verifyDelay, cmd);
}

void MyVeinsApp::applyFollowerCommand(cMessage* msg)
{
    if (!traciVehicle || isLeader) {
        delete msg;
        return;
    }

    const int followerRank = std::max(1, vehicleIndex - leaderVehicleId);
    const double desiredGap = platoonTargetGap * followerRank;
    const double leaderSpeed = msg->par("leaderSpeed").doubleValue();
    const double measuredGap = msg->par("distanceToLeader").doubleValue();
    const double gapError = measuredGap - desiredGap;

    double targetSpeed = leaderSpeed + followerControllerGain * gapError;
    targetSpeed = std::max(0.0, std::min(targetSpeed, platoonCruiseSpeed));

    traciVehicle->slowDown(targetSpeed, followerControlHorizon);
    emit(appliedSpeedSignal, targetSpeed);

    delete msg;
}

void MyVeinsApp::cleanupTimers()
{
    if (leaderBeaconTimer) {
        cancelAndDelete(leaderBeaconTimer);
        leaderBeaconTimer = nullptr;
    }
    if (leaderBrakeTimer) {
        cancelAndDelete(leaderBrakeTimer);
        leaderBrakeTimer = nullptr;
    }
}
