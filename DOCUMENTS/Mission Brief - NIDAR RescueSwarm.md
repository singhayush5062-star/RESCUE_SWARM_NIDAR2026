## Track 1 - Drone Innovation

## Mission Brief - NIDAR RescueSwarm

## Autonomous Multi-Drone Survivor Search & Aid Delivery Challenge

## Scenario

A severe flash flood has affected a semi-urban settlement located near a river basin. Roads are blocked, several houses are inaccessible, and the local rescue team has received unverified reports of stranded people across a large area of the affected zone. Telecom towers in the affected region are down, mobile connectivity is unavailable, and ground teams cannot rely on external communication networks for real-time coordination. Ground access is delayed, and the first few minutes of response are critical for locating survivors and delivering basic medical support.

The rescue command centre has identified a 10-hectare search area where up to 10 survivors may be present. Since the area is too large for a single drone to cover efficiently, innovation teams must deploy a collaborative autonomous multi-drone system that can rapidly search the area, identify survivors, geotag their locations, and deliver a survivor kit to each identified survivor.

The goal of the innovation teams is to find survivors and deliver them a survivor’s kit as quickly as possible, while ensuring that all drones operate through a single mission system, report mission data to a single operator, and complete the mission with minimal human intervention.

## 1. Overall Objective

- Teams must deploy two or more autonomous drones as part of one collaborative mission system.

- The system must scout a maximum of 10-hectare land area provided by the organisers.

- The system must identify and geotag up to 10 survivors (real humans or dummies) within the area.

- The system must deliver a survivor’s kit / medical parcel near the identified survivors.

- The mission must be executed in the shortest possible time to win while maintaining accuracy, safety, and autonomy.

## 2. Drone Configuration & Weight Constraints

- Teams may decide the number, size, and configuration of drones, subject to the minimum requirement of deploying two or more drones.

- Use of commercially available market-ready or ready-to-fly complete drone airframes shall not be permitted.

- The combined all-up weight of all deployed drones, including batteries, onboard systems, sensors, communication equipment, payload mechanisms, and any other component, must not exceed 25 kg.

- The collective weight limit shall apply to the fully deployed drone system, not to each drone individually.

- Any drone system exceeding the collective weight limit shall not be permitted to fly.


- The weight of each payload shall be 200 grams, and its dimensions shall be 20 x 10 x 5cm (l x b x h). Each payload shall be a rectangular box.

## 3. Mission Planning, Execution & Autonomy

- The organisers shall provide a mission file/boundary file for the assigned area during the setup time at the beginning of the Final Mission. This mission file will not be provided before the setup time.

- Teams shall be provided a maximum of 5 minutes for setup, configuration and mission-file loading before launch.

- Teams must load the mission file into the Mission Planner / Ground Control Station before launch.

- After the mission is initiated, the drones must autonomously:

- scout the assigned area;

- coordinate between multiple drones;

- detect survivors;

- geotag survivor locations;

- assign or execute delivery tasks;

- drop survivor’s kits near identified survivors;

- return to the launch/landing area, within a maximum flying time of 30 minutes.

- Any operator action beyond mission file loading, mission start, safety abort, or emergency recall shall be treated as manual intervention.

- Any manual waypoint change, separate drone control, flight-path adjustment, payload drop command, survivor tagging input, or mission re-planning during execution shall be considered manual intervention.

- No battery, payload, sensor, communication equipment or other component may be swapped, replaced or added after commencement of the mission.

## 4. Multi-Drone Collaboration & Mission Control

- All drones must operate as part of a coordinated system and not as independently controlled units.

- The system should demonstrate collaborative mission execution, including area division, task allocation, parallel scouting, delivery assignment, and shared mission progress reporting.

- Mission data from all drones must be reported to a single operator interface.

- Use of separate Ground Control Stations, or independent control systems for different drones, shall not be permitted.

- The Mission Planner / Ground Control Station must display:

- loading of the mission file/mission boundary;

- live location and status of all deployed drones;

- tagged location of detected survivors;

- delivery status of survivor’s kits by each drone;

- basic mission progress and completion status.

- Live video feed display


- Any system input, command, or operator action outside the above-permitted functions shall be considered manual intervention, except for a safety abort or emergency recall.

## 5. Communication & Network Constraint

- The mission shall be designed and executed assuming a no-external-network environment.

- Teams shall not rely on GSM, LTE, 5G, public Wi-Fi, internet connectivity, cloud-based communication, or any external network for drone operations, mission execution, data transfer, or coordination.

- All communication between drones and the Mission Planner/Ground Control Station must be handled through the team’s local communication link or onboard autonomy system. Optical-fibre cables, wired links, physical tethers or any other flight-connected cable shall not be permitted.

- The system must operate assuming there is no mobile network or internet connectivity.

- Use of any external network-based communication interface during the mission shall be treated as a violation or manual/external intervention, as applicable.

## 6. Team Deployment & Human Intervention

- A maximum of two team members shall be allowed for setting up drones, payloads, communication systems, and associated equipment during the permitted 5-minute setup period.

- No more than one operator shall supervise the mission through the Ground Control Station.

- No assistance of any kind shall be allowed from any other team member during setup, launch, mission execution, payload delivery, landing, troubleshooting, or recovery.

- Other team members may be present only as observers and shall not provide verbal, physical, digital, or technical assistance during the mission.

## 7. Launch, Landing & Field Constraints

- All drones must be launched from and land within a fixed 12 ft x 12 ft launch/landing area.

- The mission shall be conducted outdoors under standard daylight conditions.

- No part of any drone shall cross or remain outside the launch/landing area during launch or landing.

- The drones must operate only within the assigned mission boundary.

- Landing outside the designated launch/landing area shall be treated as a violation, except in case of an emergency landing triggered for safety reasons.

## 8. Safety & Failsafe Requirements

- Each drone must have return-to-home capability.

- Each drone must include failsafe features for: low battery; loss of command and control link; geofence breach; mission abort; emergency recall.

- The system must allow the operator to safely abort the mission and recall all drones if required.
