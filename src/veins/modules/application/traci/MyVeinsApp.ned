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

package org.car2x.veins.modules.application.traci;
import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

//
// network description file for your Veins Application. Add parameters here
//
simple MyVeinsApp extends DemoBaseApplLayer
{
    @class(veins::MyVeinsApp);
    string appName = default("PQC Platooning App");

    int platoonLeaderId = default(0);
    int platoonBeaconPsid = default(77);

    double platoonTargetGap = default(15); // meters between consecutive vehicles
    double platoonCruiseSpeed = default(25); // m/s before emergency brake
    double followerControllerGain = default(0.35);
    volatile double followerControlHorizon @unit(s) = default(0.2s);

    volatile double leaderBeaconInterval @unit(s) = default(0.1s);
    volatile double leaderBrakeTime @unit(s) = default(20s);
    volatile double leaderBrakeDuration @unit(s) = default(2s);
    double leaderBrakeTargetSpeed = default(0); // m/s

    int pqcSignatureBytes = default(2420); // Dilithium-like signature size
    int pqcPublicKeyBytes = default(1312); // Dilithium public key size
    volatile double pqcSignDelay @unit(s) = default(2ms);
    volatile double pqcVerifyDelay @unit(s) = default(5ms);
    volatile double pqcDelayJitter @unit(s) = default(0s);
    bool randomizePqcDelay = default(false);
}
