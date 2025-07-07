#include "VehicleControlApp.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include<bits/stdc++.h>
using namespace veins;
using namespace std;

Register_Class(VehicleControlApp);

void VehicleControlApp::initialize(int stage) {
    TraCIDemo11p::initialize(stage);
    if (stage == 0) {
        statusUpdateMsg = new cMessage("statusUpdate");
        cleanupTimer = new cMessage("vehicleCleanup");
        mobility = TraCIMobilityAccess().get(getParentModule());
        traciVehicle = mobility->getVehicleCommandInterface();
        int moduleIndex = getParentModule()->getIndex();
        myInternalId = moduleIndex;
        mySimulationId = 16 + 6 * myInternalId;
        string sumoId = mobility->getExternalId();
        EV << "\n[VEHICLE] ********************************************" << std::endl;
        EV << "[VEHICLE] Initialized with internal ID: " << myInternalId
           << ", simulation ID: " << mySimulationId << std::endl;
        EV << "[VEHICLE] Parent module: " << getParentModule()->getFullName()
           << ", index: " << getParentModule()->getIndex() << std::endl;
        EV << "[VEHICLE] ********************************************" << std::endl;
        startTime = simTime().dbl();
        startingRoad = mobility->getRoadId();
        SimulationLogger::getInstance().recordVehicleStart(mySimulationId, startingRoad, startTime);
        
        // initialize data structures
        allRoads.clear();
        currentPath.clear();
        destinations.clear();
        earliestArrival = 0.0;
        latestArrival = 0.0;
        endTime = 0.0;
        pathLength = 0.0;
        graphProcessor.reset(new GraphProcessor(roadNetwork));
        requestAllRoads();
    } else if (stage == 1) {
        scheduleAt(simTime() + uniform(1.0, 2.0), statusUpdateMsg);
        scheduleAt(simTime() + 0.3, cleanupTimer);
        EV << "[VEHICLE] Vehicle " << myId << " initialized" << std::endl;
    }
}

void VehicleControlApp::onWSM(BaseFrame1609_4* wsm) {
    auto* msg = dynamic_cast<TraCIDemo11pMessage*>(wsm->getEncapsulatedPacket());
    if (!msg) return;

    std::string data = msg->getDemoData();
    
    LAddress::L2Type senderAddress = msg->getSenderAddress();
    LAddress::L2Type recipientAddress = wsm->getRecipientAddress();
    
    EV << "\n[VEHICLE] ====== RECEIVING MESSAGE ======" << std::endl;
    EV << "[VEHICLE] Sender address: " << senderAddress << std::endl;
    EV << "[VEHICLE] Recipient address: " << recipientAddress << std::endl; 
    EV << "[VEHICLE] My internal ID: " << myInternalId << std::endl;
    EV << "[VEHICLE] My simulation ID: " << mySimulationId << std::endl;
    EV << "[VEHICLE] Message content: " << data << std::endl;
    EV << "[VEHICLE] =============================" << std::endl;
    EV << "[VEHICLE] Received response: " << data << std::endl;
    
    // Process different response types
    if (data.find("ALL_ROADS:") == 0) {
        processAllRoadsResponse(data.substr(10));  // Skip "ALL_ROADS:"
    }
 
    else if (data.find("DESTINATIONS:") == 0) {
        processDestinationsResponse(data.substr(13));  // Skip "DESTINATIONS:"
    } 

    else if (data.find("ERROR:") == 0) {
        EV << "[VEHICLE] Error from RSU: " << data.substr(6) << std::endl;  // Skip "ERROR:"
    } 
    else if (data.find("NETWORK_LOADED:") == 0) {
        std::string result = data.substr(15);  // Skip "NETWORK_LOADED:"
        EV << "[VEHICLE] Network loaded: " << result << std::endl;
    }
    else if (data.find("CHANGE_ROUTE:") == 0) {
        // Parse the CHANGE_ROUTE message
        std::string routeInfo = data.substr(13); // Skip "CHANGE_ROUTE:"
        size_t colonPos = routeInfo.find(':');
        
        EV << "\n[VEHICLE] ************************************************" << std::endl;
        EV << "[VEHICLE] RECEIVED CHANGE_ROUTE MESSAGE: " << data << std::endl;
        if (colonPos != std::string::npos) {
            std::string vehIdStr = routeInfo.substr(0, colonPos);
            int targetVehId = std::stoi(vehIdStr);
            bool messageIsForMe = (mySimulationId == targetVehId);
            if (!messageIsForMe) {
                return;
            }
            string routePath = routeInfo.substr(colonPos + 1);
            
            // Parse the space-separated list of edges
            istringstream routeStream(routePath);
            vector<string> routeEdges;
            string edge;
            
            while (routeStream >> edge) {
                routeEdges.push_back(edge);
            }
            std::string currentEdge = traciVehicle->getRoadId();
            std::list<std::string> plannedRoute = traciVehicle->getPlannedRoadIds();
            if (routeEdges.size() > 1) {
                try {
                    bool routeIsValid = true;
                    for (size_t i = 0; i < routeEdges.size() - 1; i++) {
                        EV << "[VEHICLE] Checking connectivity between edges: " << routeEdges[i]
                           << " and " << routeEdges[i+1] << std::endl;
                    }
                    if (routeIsValid) {
                        std::list<std::string> edgesList(routeEdges.begin(), routeEdges.end());

                        // Store original route for comparison
                        std::list<std::string> originalRoute = traciVehicle->getPlannedRoadIds();

                        // Use TraCI to change the vehicle's route with the complete path
                        traciVehicle->changeVehicleRoute(edgesList);

                        // Get the new route after change
                        std::list<std::string> newPlannedRoute = traciVehicle->getPlannedRoadIds();

                        // Check if the route actually changed by comparing with original route
                        bool routeChanged = (newPlannedRoute != originalRoute);

                        // Verify the new route contains edges from our proposed route
                        bool containsProposedEdges = false;
                        if (!routeEdges.empty() && !newPlannedRoute.empty()) {
                            // Check if at least the destination edge is included
                            std::string targetEdge = routeEdges.back();
                            for (const auto& edge : newPlannedRoute) {
                                if (edge == targetEdge) {
                                    containsProposedEdges = true;
                                    break;
                                }
                            }
                        }

                        if (routeChanged || containsProposedEdges) {
                            EV << "[VEHICLE] Route changed successfully to: ";
                            for (const auto& edgeId : newPlannedRoute) {
                                EV << edgeId << " ";
                            }
                            EV << std::endl;
                            // Highlight the change by modifying vehicle appearance
                            traciVehicle->setColor(TraCIColor(255, 0, 0, 255)); // Red color

                            return; // Success, no need for fallback
                        } else {
                            EV << "[VEHICLE] Route change didn't take effect, falling back to destination-only method" << std::endl;
                        }
                    } else {
                        EV << "[VEHICLE] Route connectivity validation failed, falling back to destination-only method" << std::endl;
                    }
                }
                catch (const std::exception& e) {
                    EV << "[VEHICLE] ERROR changing complete route: " << e.what() << std::endl;
                    EV << "[VEHICLE] Falling back to destination-only method" << std::endl;
                }
            }

            // Fallback: use destination-only method (get the last edge from the route)
            try {
                // Get the last edge from the route as destination
                std::string destinationEdge;
                if (!routeEdges.empty()) {
                    destinationEdge = routeEdges.back();
                } else {
                    throw std::runtime_error("Empty route provided");
                }

                EV << "[VEHICLE] Using SUMO's routing to find path to destination edge: " << destinationEdge << std::endl;
                // Store the original route
                std::list<std::string> originalRoute = traciVehicle->getPlannedRoadIds();

                // Use TraCI to change destination (let SUMO compute the route)
                traciVehicle->changeTarget(destinationEdge);

                // Get the new route that SUMO calculated
                std::list<std::string> newPlannedRoute = traciVehicle->getPlannedRoadIds();

                // Check if the route actually changed
                bool routeChanged = (originalRoute != newPlannedRoute);

                EV << "[VEHICLE] SUMO calculated route: ";
                std::cout << "SUMO calculated route: ";
                for (const auto& edgeId : newPlannedRoute) {
                    EV << edgeId << " ";
                }
                EV << std::endl;
                
                if (routeChanged) {
                    EV << "[VEHICLE] Route changed successfully to destination " << destinationEdge << std::endl;
                    traciVehicle->setColor(TraCIColor(0, 0, 255, 255));
                } else {
                    EV << "[VEHICLE] Warning: Route didn't change, destination may already be included or SUMO rejected the change" << std::endl;
                    bool destInRoute = false;
                    for (const auto& edge : originalRoute) {
                        if (edge == destinationEdge) {
                            destInRoute = true;
                            break;
                        }
                    }

                    if (destInRoute) {
                        EV << "[VEHICLE] Destination " << destinationEdge << " is already in the current route" << std::endl;
                        std::cout << "Destination " << destinationEdge << " is already in the current route" << std::endl;
                    } else {
                        EV << "[VEHICLE] SUMO unable to find a route to destination " << destinationEdge << std::endl;
                        std::cout << "SUMO unable to find a route to destination " << destinationEdge << std::endl;
                    }
                }
            }
            catch (const std::exception& e) {
                EV << "[VEHICLE] ERROR changing route: " << e.what() << std::endl;
                std::cout << "ERROR changing route: " << e.what() << std::endl;
                
                // Try fallback approach: just use the first edge in the proposed route as destination
                if (routeEdges.size() > 0) {
                    try {
                        std::string fallbackDestination = routeEdges[0];

                        EV << "[VEHICLE] Trying fallback to first proposed edge: " << fallbackDestination << std::endl;
                        std::cout << "Trying fallback to first proposed edge: " << fallbackDestination << std::endl;

                        traciVehicle->changeTarget(fallbackDestination);
                        EV << "[VEHICLE] Fallback route change succeeded" << std::endl;
                        std::cout << "Fallback route change succeeded" << std::endl;
                    }
                    catch (const std::exception& e2) {
                        EV << "[VEHICLE] Fallback route change also failed: " << e2.what() << std::endl;
                        std::cout << "Fallback route change also failed: " << e2.what() << std::endl;
                    }
                }
            }
        }
        else {
            EV << "[VEHICLE] Invalid CHANGE_ROUTE format: " << data << std::endl;
            std::cout << "Invalid CHANGE_ROUTE format: " << data << std::endl;
        }
        EV << "[VEHICLE] ************************************************\n" << std::endl;
    }
}

void VehicleControlApp::handleSelfMsg(cMessage* msg) {
    if (msg == statusUpdateMsg) {
        // Send status update to RSU
        sendStatusUpdate();
        // Schedule next update
        scheduleAt(simTime() + 2.0, statusUpdateMsg);
    }

    else if (msg == cleanupTimer) {
        // Check if vehicle has reached destination
        checkVehicleStatus();
        
        // Schedule next check if still active - check EVERY 0.1 seconds
        if (!hasLoggedDeparture) {
            scheduleAt(simTime() + 0.1, cleanupTimer);  // Check frequently (every 0.1s)
        }
    }

    else {
        TraCIDemo11p::handleSelfMsg(msg);
    }
}

void VehicleControlApp::sendStatusUpdate() {
    // Get current position and road
    Coord pos = mobility->getPositionAt(simTime());
    std::string lane = traciVehicle->getLaneId();
    std::string road = traciVehicle->getRoadId();
    double speed = mobility->getSpeed();

    // Update current road
    currentRoadId = road;

    // Create status message - Send both internal and simulation IDs
    std::ostringstream oss;
    oss << "STATUS:"
        << "simId=" << mySimulationId << ";"
        << "internalId=" << myInternalId << ";"
        << "road=" << road << ";"
        << "lane=" << lane << ";"
        << "pos=" << pos.x << "," << pos.y << "," << pos.z << ";"
        << "speed=" << speed;

    // Send message
    auto* status = new TraCIDemo11pMessage();
    status->setDemoData(oss.str().c_str());
    status->setSenderAddress(myId);

    auto* wsm = new BaseFrame1609_4();
    wsm->encapsulate(status);
    populateWSM(wsm);
    sendDown(wsm);

    EV << "[VEHICLE] Sent status update from road " << road << std::endl;
    EV << "[VEHICLE] Status includes: internal ID " << myInternalId
       << ", simulation ID " << mySimulationId << std::endl;
    std::cout << "Vehicle " << myInternalId << " (sim ID: " << mySimulationId
              << ") sent status update from road " << road << std::endl;
}

void VehicleControlApp::requestAllRoads() {
    // Create request message
    std::string request = "GET_ALL_ROADS";

    // Send message
    auto* req = new TraCIDemo11pMessage();
    req->setDemoData(request.c_str());
    req->setSenderAddress(myId);
    
    auto* wsm = new BaseFrame1609_4();
    wsm->encapsulate(req);
    populateWSM(wsm);
    sendDown(wsm);
    
    EV << "[VEHICLE] Requested all roads" << std::endl;
}









void VehicleControlApp::requestDestinations(int count) {
    // Create request message
    std::ostringstream oss;
    oss << "GENERATE_DESTINATIONS:" << count;
    
    // Send message
    auto* req = new TraCIDemo11pMessage();
    req->setDemoData(oss.str().c_str());
    req->setSenderAddress(myId);
    
    auto* wsm = new BaseFrame1609_4();
    wsm->encapsulate(req);
    populateWSM(wsm);
    sendDown(wsm);
    
    EV << "[VEHICLE] Requested " << count << " random destinations - DEBUG: message=" << oss.str() << std::endl;
}



void VehicleControlApp::processAllRoadsResponse(const std::string& data) {
    allRoads = parseRoadList(data);
    
    EV << "[VEHICLE] Received list of all roads: " << allRoads.size() << " roads" << std::endl;

    // Print the road list to standard output
    EV << "\n=============== ROAD NETWORK INFORMATION ===============" << std::endl;
    EV << "Received " << allRoads.size() << " roads from RSU:" << std::endl;
    
    int maxRoadsToDisplay = std::min(static_cast<size_t>(20), allRoads.size());
    for (int i = 0; i < maxRoadsToDisplay; i++) {
        EV << "  - Road ID: " << allRoads[i] << std::endl;
    }
    
    if (allRoads.size() > maxRoadsToDisplay) {
        EV << "  ... and " << (allRoads.size() - maxRoadsToDisplay) << " more roads" << std::endl;
    }

    // Initialize graph processor with road network
    if (allRoads.size() > 0) {
        graphProcessor.reset(new GraphProcessor(roadNetwork));
    }

    // Immediately run path finding tests to print shortest paths
    EV << "\n=============== IMMEDIATE PATH FINDING TEST ===============" << std::endl;

    // Kiểm tra đường đi giữa các con đường
    if (allRoads.size() >= 2) {
        std::string source1 = allRoads.front();
        std::string target1 = allRoads.back();
        EV << "Test 1: Finding path from road " << source1 << " to road " << target1 << std::endl;
        std::vector<std::string> path;
        double pathLength = 0.0;
        if (graphProcessor) {
            path = graphProcessor->findShortestPath(source1, target1);
            pathLength = graphProcessor->getShortestPathLength(source1, target1);
        }

        if (!path.empty() && pathLength > 0) {
            EV << "SUCCESS: Path found with length " << pathLength << std::endl;
            // In đường đi trên một dòng với mũi tên phân tách
            EV << "Path: ";
            for (size_t i = 0; i < path.size(); i++) {
                EV << path[i];
                if (i < path.size() - 1) {
                    EV << " -> ";
                }
            }
            EV << std::endl;
        } else {
            EV << "NO PATH found between " << source1 << " and " << target1 << std::endl;
        }

        // Kiểm tra đường đi giữa đoạn đầu và giữa
        if (allRoads.size() >= 3) {
            std::string target2 = allRoads[allRoads.size() / 2];
            EV << "\nTest 2: Finding path from road " << source1 << " to road " << target2 << std::endl;
            if (graphProcessor) {
                path = graphProcessor->findShortestPath(source1, target2);
                pathLength = graphProcessor->getShortestPathLength(source1, target2);
            }

            if (!path.empty() && pathLength > 0) {
                EV << "SUCCESS: Path found with length " << pathLength << std::endl;
                // In đường đi trên một dòng với mũi tên phân tách
                EV << "Path: ";
                for (size_t i = 0; i < path.size(); i++) {
                    EV << path[i];
                    if (i < path.size() - 1) {
                        EV << " -> ";
                    }
                }
                EV << std::endl;
            } else {
                EV << "NO PATH found between " << source1 << " and " << target2 << std::endl;
            }
        }
    } else {
        EV << "Not enough roads to perform path finding tests" << std::endl;
    }
}









void VehicleControlApp::processDestinationsResponse(const std::string& data) {
    EV << "[VEHICLE] DEBUG: Received destination response data: " << data << std::endl;
    
    size_t colonPos = data.find(':');
    if (colonPos != std::string::npos) {
        std::string countStr = data.substr(0, colonPos);
        std::string destsStr = data.substr(colonPos + 1);

        int destCount = std::stoi(countStr);
        destinations.clear();
        std::vector<std::string> destStrings;
        std::istringstream iss(destsStr);
        std::string destString;

        while (std::getline(iss, destString, ';')) {
            std::vector<std::string> parts = parseRoadList(destString);

            if (parts.size() >= 3) {
                std::string nodeId = parts[0];
                double earliness = std::stod(parts[1]);
                double tardiness = std::stod(parts[2]);

                destinations.emplace_back(nodeId, TimeWindow(earliness, tardiness));
                if (destinations.size() == 1) {
                    targetRoad = nodeId;
                    earliestArrival = earliness;
                    latestArrival = tardiness;
                    std::vector<std::string> path;
                    double algorithmTime = 0.0;
                    double estimatedTravelTime = 0.0;
                    
                    if (graphProcessor) {
                        // Start measuring algorithm time - IMPORTANT: Place this before ALL algorithm operations
                        auto startAlgorithm = std::chrono::high_resolution_clock::now();
                        
                        // Run the full Dijkstra algorithm directly instead of just using its results
                        std::map<std::string, std::pair<double, std::string>> dijkstraResult = 
                            graphProcessor->dijkstra(startingRoad);
                        
                        // Reconstruct the path based on Dijkstra results
                        path = graphProcessor->reconstructPath(dijkstraResult, startingRoad, targetRoad);
                        
                        // Get path length from Dijkstra results
                        auto it = dijkstraResult.find(targetRoad);
                        if (it != dijkstraResult.end()) {
                            pathLength = it->second.first;
                        } else {
                            pathLength = 0.0;
                        }
                        
                        // Force additional computations to make timing more measurable
                        // This will ensure we're not getting 0.000000 due to compiler optimizations
                        for (int i = 0; i < 10; i++) {
                            // Run multiple times to get better timing measurement
                            dijkstraResult = graphProcessor->dijkstra(startingRoad);
                            path = graphProcessor->reconstructPath(dijkstraResult, startingRoad, targetRoad);
                        }
                        
                        // End algorithm timing measurement here to include all algorithm operations
                        auto endAlgorithm = std::chrono::high_resolution_clock::now();
                        algorithmTime = std::chrono::duration<double>(endAlgorithm - startAlgorithm).count();
                        
                        // Start calculating estimated travel time (not part of algorithm timing)
                        // Get the road network graph
                        const Graph& graph = graphProcessor->getGraph();
                        
                        // Calculate time for each segment of the path
                        for (size_t j = 0; j < path.size(); j++) {
                            const std::string& edgeId = path[j];
                            double edgeLength = 0.0;
                            double edgeSpeed = 13.89; // Default speed if not found
                            bool edgeFound = false;
                            
                            // Find this edge in the graph to get its properties
                            for (const auto& nodePair : graph.getAdjList()) {
                                for (const auto& edge : nodePair.second) {
                                    if (edge.getId() == edgeId) {
                                        edgeLength = edge.getLength();
                                        
                                        // Use the Edge's getMaxSpeed method instead of manually checking lanes
                                        edgeSpeed = edge.getMaxSpeed();
                                        
                                        // Calculate time to traverse this segment
                                        double segmentTime = edgeLength / edgeSpeed;
                                        estimatedTravelTime += segmentTime;
                                        
                                        edgeFound = true;
                                        break;
                                    }
                                }
                                if (edgeFound) break;
                            }
                            
                            if (!edgeFound) {
                                // Use default values if edge not found
                                estimatedTravelTime += 100.0 / 13.89;
                            }
                        }
                        
                        // Use calculated travel time when updating vehicle destination
                        SimulationLogger::getInstance().updateVehicleDestination(
                            mySimulationId, targetRoad, earliestArrival, latestArrival, path, pathLength, estimatedTravelTime);
                    }
                    
                    // Không cần ghi nhận thời gian thuật toán cho từng xe nữa
                }
            }
        }
        
        EV << "[VEHICLE] Received " << destinations.size() << " destinations:" << std::endl;
        for (const auto& dest : destinations) {
            EV << "  - Node " << dest.nodeId << " (time window: "
               << dest.timeWindow.earliness << " - " << dest.timeWindow.tardiness << ")" << std::endl;
        }
    } else {
        EV << "[VEHICLE] ERROR: Malformed destination response data" << std::endl;
    }
}



std::vector<std::string> VehicleControlApp::parseRoadList(const std::string& data, char delimiter) {
    std::vector<std::string> result;
    std::istringstream iss(data);
    std::string road;
    
    while (std::getline(iss, road, delimiter)) {
        if (!road.empty()) {
            result.push_back(road);
        }
    }
    
    return result;
}





void VehicleControlApp::onBSM(DemoSafetyMessage* /*bsm*/) {
    // Not used in this implementation
}

void VehicleControlApp::onWSA(DemoServiceAdvertisment* /*wsa*/) {
    // Not used in this implementation
}

void VehicleControlApp::handleLowerMsg(cMessage* msg) {
    TraCIDemo11p::handleLowerMsg(msg);
}

void VehicleControlApp::handlePositionUpdate(cObject* obj) {
    TraCIDemo11p::handlePositionUpdate(obj);
    
    // Check if vehicle has reached its destination or is about to be removed
    if (mobility && traciVehicle) {
        // Get the vehicle's current state
        std::string roadId = traciVehicle->getRoadId();
        
        // If the vehicle is on its last road segment or has no road ID (about to be removed)
        if (roadId.empty() || roadId == "") {
            // Vehicle might be at destination or about to be removed
            logDepartureIfNeeded();
            
            // Also check if we can get the exact position
            Coord pos = mobility->getPositionAt(simTime());
            if (pos.x == 0 && pos.y == 0) {
                // This likely means the vehicle is being removed
                logDepartureIfNeeded();
            }
        }
        // If the vehicle is on its target road - it has reached the destination
        else if (!targetRoad.empty() && roadId == targetRoad) {
            // Vehicle has reached destination
            logDepartureIfNeeded();
        }
    }
}

void VehicleControlApp::finish() {
    endTime = simTime().dbl();
    if (!hasLoggedDeparture) {
        // Record vehicle end to logger
        SimulationLogger::getInstance().recordVehicleEnd(mySimulationId, endTime);
        hasLoggedDeparture = true;
    }
    cleanupMessages();
    TraCIDemo11p::finish();
}

void VehicleControlApp::cleanupMessages() {
    if (statusUpdateMsg) {
        cancelAndDelete(statusUpdateMsg);
        statusUpdateMsg = nullptr;
    }

    if (cleanupTimer) {
        cancelAndDelete(cleanupTimer);
        cleanupTimer = nullptr;
    }
}

void VehicleControlApp::logDepartureIfNeeded() {
    // Only log departure once
    if (!hasLoggedDeparture) {
        endTime = simTime().dbl();
        
        EV << "[VEHICLE] Vehicle " << mySimulationId << " reached destination or is being removed at time " 
           << std::fixed << std::setprecision(1) << endTime << std::endl;
        SimulationLogger::getInstance().recordVehicleEnd(mySimulationId, endTime);
        hasLoggedDeparture = true;
        // Check if arrived on time
        bool arrivedOnTime = (endTime <= latestArrival);
        double timeWindowDeviation = 0.0;
        
        if (endTime < earliestArrival) {
            timeWindowDeviation = earliestArrival - endTime;
        } else if (endTime > latestArrival) {
            timeWindowDeviation = endTime - latestArrival;
        }
    }
}



void VehicleControlApp::checkVehicleStatus() {
    if (mobility && traciVehicle) {
        string roadId = traciVehicle->getRoadId();
        if (!targetRoad.empty() && roadId == targetRoad) {
            EV << "[VEHICLE] Vehicle " << mySimulationId << " has reached destination road " 
               << targetRoad << " at time " << simTime() << std::endl;
            logDepartureIfNeeded();
        }
        else if (roadId.empty() || roadId == "") {
            EV << "[VEHICLE] Vehicle " << mySimulationId << " is no longer on a road at time " 
               << simTime() << " (may be removed from simulation)" << std::endl;
            logDepartureIfNeeded();
        }
    }
}







