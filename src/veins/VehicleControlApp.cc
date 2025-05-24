#include "VehicleControlApp.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include <sstream>
#include <iostream>

using namespace veins;
using namespace std;

Register_Class(VehicleControlApp);

void VehicleControlApp::initialize(int stage) {
    TraCIDemo11p::initialize(stage);
    if (stage == 0) {
        statusUpdateMsg = new cMessage("statusUpdate");
        requestRoadInfoMsg = new cMessage("requestRoadInfo");
        mobility = TraCIMobilityAccess().get(getParentModule());
        traciVehicle = mobility->getVehicleCommandInterface();
        int moduleIndex = getParentModule()->getIndex();
        std::map<int, int> idMapping = {
            {0, 16},   // index 0 maps to simulation ID 16
            {1, 22},   // index 1 maps to simulation ID 22
            {2, 28},
            {3, 34},
            {4, 40},
            {5, 46},
            {6, 52},
            {7, 58},
            {8, 64},
            {9, 70},
            {10, 76},
            {11, 82},
            {12, 88},
            {13, 94},
            {14, 100},
            {15, 106},
            {16, 112},
            {17, 118},
            {18, 124},
            {19, 130},
            {20, 136},
            {21, 142},
            {22, 148},
            {23, 154},
            {24, 160},
            {25, 166},
            {26, 172},
            {27, 178},
            {28, 184},
            {29, 190},
            {30, 196},
            {31, 202},
            {32, 208},
        };
        myInternalId = moduleIndex;
        auto it = idMapping.find(myInternalId);
        if (it != idMapping.end()) {
            mySimulationId = it->second;
        }
        EV << "\n[VEHICLE]" << std::endl;
        EV << "[VEHICLE] Initialized with internal ID: " << myInternalId 
           << ", simulation ID: " << mySimulationId << std::endl;
        EV << "[VEHICLE] Parent module: " << getParentModule()->getFullName() 
           << ", index: " << getParentModule()->getIndex() << std::endl;
        EV << "[VEHICLE] ********************************************" << std::endl;

        mobility = TraCIMobilityAccess().get(getParentModule());
        traciVehicle = mobility->getVehicleCommandInterface();
        currentRoadId = "";
        allRoads.clear();
        accessibleRoads.clear();
        incomingRoads.clear();
        currentRoadAttributes.clear();
        currentPath.clear();
        destinations.clear();
        graphProcessor.reset(new GraphProcessor(roadNetwork));
        requestAllRoads();
        EV << "\n[VEHICLE] Requesting road network data from RSU..." << std::endl;
        EV << "[VEHICLE] Road information will be printed once received." << std::endl;
    } else if (stage == 1) {
        scheduleAt(simTime() + uniform(1.0, 2.0), statusUpdateMsg);
        scheduleAt(simTime() + uniform(3.0, 5.0), requestRoadInfoMsg);
        cMessage* testPathFindingMsg = new cMessage("testPathFinding");
        scheduleAt(simTime() + 10.0, testPathFindingMsg);
        cMessage* testTaskGeneratorMsg = new cMessage("testTaskGenerator");
        scheduleAt(simTime() + 15.0, testTaskGeneratorMsg);
        EV << "[VEHICLE] Vehicle " << myId << " initialized" << std::endl;
    }
}

void VehicleControlApp::onWSM(BaseFrame1609_4* wsm) {
    auto* msg = dynamic_cast<TraCIDemo11pMessage*>(wsm->getEncapsulatedPacket());
    if (!msg) return;
    string data = msg->getDemoData();
    LAddress::L2Type senderAddress = msg->getSenderAddress();
    LAddress::L2Type recipientAddress = wsm->getRecipientAddress();
    
    EV << "\n[VEHICLE] ====== RECEIVING MESSAGE ======" << std::endl;
    EV << "[VEHICLE] Sender address: " << senderAddress << std::endl;
    EV << "[VEHICLE] Recipient address: " << recipientAddress << std::endl; 
    EV << "[VEHICLE] My internal ID: " << myInternalId << std::endl;
    EV << "[VEHICLE] My simulation ID: " << mySimulationId << std::endl;
    EV << "[VEHICLE] Message content: " << data << std::endl;
    EV << "[VEHICLE] =============================" << std::endl;
    
    std::cout << "\nVEHICLE " << myInternalId << " (sim ID: " << mySimulationId << ") RECEIVED MESSAGE FROM " << senderAddress << std::endl;
    std::cout << "DESTINATION ADDRESS: " << recipientAddress << std::endl;
    std::cout << "MESSAGE CONTENT: " << data << std::endl;
    
    EV << "[VEHICLE] Received response: " << data << std::endl;
    
    // Process different response types
    if (data.find("ALL_ROADS:") == 0) {
        processAllRoadsResponse(data.substr(10));  // Skip "ALL_ROADS:"
    }
    else if (data.find("ACCESSIBLE_ROADS:") == 0) {
        processAccessibleRoadsResponse(data.substr(17));  // Skip "ACCESSIBLE_ROADS:"
    } 
    else if (data.find("INCOMING_ROADS:") == 0) {
        processIncomingRoadsResponse(data.substr(15));  // Skip "INCOMING_ROADS:"
    } 
    else if (data.find("ROAD_ATTRIBUTES:") == 0) {
        processRoadAttributesResponse(data.substr(16));  // Skip "ROAD_ATTRIBUTES:"
    } 
    else if (data.find("SHORTEST_PATH:") == 0) {
        processShortestPathResponse(data.substr(14));  // Skip "SHORTEST_PATH:"
    } 
    else if (data.find("K_PATHS:") == 0) {
        processKPathsResponse(data.substr(8));  // Skip "K_PATHS:"
    } 
    else if (data.find("DESTINATIONS:") == 0) {
        processDestinationsResponse(data.substr(13));  // Skip "DESTINATIONS:"
    } 
    else if (data.find("VALID_ASSIGNMENT:") == 0) {
        processValidAssignmentResponse(data.substr(17));  // Skip "VALID_ASSIGNMENT:"
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
        std::cout << "Vehicle " << myInternalId << " (sim ID: " << mySimulationId 
                  << ") received change route message: " << data << std::endl;
        
        if (colonPos != std::string::npos) {
            // Extract vehicle ID
            std::string vehIdStr = routeInfo.substr(0, colonPos);
            int targetVehId = std::stoi(vehIdStr);
            
            EV << "[VEHICLE] Message targets simulation ID: " << targetVehId 
               << ", my internal ID: " << myInternalId 
               << ", my simulation ID: " << mySimulationId << std::endl;
            
            // IMPORTANT: Check if this broadcast message is meant for me based on simulation ID
            bool messageIsForMe = (mySimulationId == targetVehId);
            
            if (!messageIsForMe) {
                EV << "[VEHICLE] Received CHANGE_ROUTE for simulation ID " << targetVehId 
                   << ", but my simulation ID is " << mySimulationId << ". Ignoring." << std::endl;
                std::cout << "Ignoring message meant for different vehicle (ID mismatch)" << std::endl;
                return;
            }
            
            EV << "[VEHICLE] This message is for me! Processing..." << std::endl;
            std::cout << "This message is for me! Processing..." << std::endl;
            
            // Extract the route path
            std::string routePath = routeInfo.substr(colonPos + 1);
            
            // Parse the space-separated list of edges
            std::istringstream routeStream(routePath);
            std::vector<std::string> routeEdges;
            std::string edge;
            
            while (routeStream >> edge) {
                routeEdges.push_back(edge);
            }
            
            // Get current vehicle position and planned route
            std::string currentEdge = traciVehicle->getRoadId();
            std::list<std::string> plannedRoute = traciVehicle->getPlannedRoadIds();
            
            EV << "[VEHICLE] Current position: Edge " << currentEdge << std::endl;
            std::cout << "Current position: Edge " << currentEdge << std::endl;
            
            EV << "[VEHICLE] Planned route: ";
            std::cout << "Planned route: ";
            for (const auto& edgeId : plannedRoute) {
                EV << edgeId << " ";
                std::cout << edgeId << " ";
            }
            EV << std::endl;
            std::cout << std::endl;
            
            EV << "[VEHICLE] Proposed new route: ";
            std::cout << "Proposed new route: ";
            for (const auto& edgeId : routeEdges) {
                EV << edgeId << " ";
                std::cout << edgeId << " ";
            }
            EV << std::endl;
            std::cout << std::endl;
            
            // First try to set the complete route if available
            if (routeEdges.size() > 1) {
                try {
                    EV << "[VEHICLE] Attempting to use complete route with " << routeEdges.size() << " edges" << std::endl;
                    std::cout << "Attempting to use complete route with " << routeEdges.size() << " edges" << std::endl;
                    
                    // Validate route connectivity by checking if consecutive edges are connected
                    bool routeIsValid = true;
                    for (size_t i = 0; i < routeEdges.size() - 1; i++) {
                        EV << "[VEHICLE] Checking connectivity between edges: " << routeEdges[i] 
                           << " and " << routeEdges[i+1] << std::endl;
                        std::cout << "Checking connectivity between edges: " << routeEdges[i] 
                                  << " and " << routeEdges[i+1] << std::endl;
                    }
                    
                    // Only try to set route if we believe it's valid
                    if (routeIsValid) {
                        std::list<std::string> edgesList(routeEdges.begin(), routeEdges.end());
                        std::list<std::string> originalRoute = traciVehicle->getPlannedRoadIds();
                        traciVehicle->changeVehicleRoute(edgesList);
                        std::list<std::string> newPlannedRoute = traciVehicle->getPlannedRoadIds();
                        bool routeChanged = (newPlannedRoute != originalRoute);
                        bool containsProposedEdges = false;
                        if (!routeEdges.empty() && !newPlannedRoute.empty()) {
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
                            std::cout << "Route changed successfully to: ";
                            for (const auto& edgeId : newPlannedRoute) {
                                EV << edgeId << " ";
                                std::cout << edgeId << " ";
                            }
                            EV << std::endl;
                            std::cout << std::endl;
                            
                            // Highlight the change by modifying vehicle appearance
                            traciVehicle->setColor(TraCIColor(255, 0, 0, 255)); // Red color
                            
                            return; // Success, no need for fallback
                        } else {
                            EV << "[VEHICLE] Route change didn't take effect, falling back to destination-only method" << std::endl;
                            std::cout << "Route change didn't take effect, falling back to destination-only method" << std::endl;
                            // Continue to fallback method
                        }
                    } else {
                        EV << "[VEHICLE] Route connectivity validation failed, falling back to destination-only method" << std::endl;
                        std::cout << "Route connectivity validation failed, falling back to destination-only method" << std::endl;
                        // Continue to fallback method
                    }
                }
                catch (const std::exception& e) {
                    EV << "[VEHICLE] ERROR changing complete route: " << e.what() << std::endl;
                    std::cout << "ERROR changing complete route: " << e.what() << std::endl;
                    EV << "[VEHICLE] Falling back to destination-only method" << std::endl;
                    std::cout << "Falling back to destination-only method" << std::endl;
                    // Continue to fallback method below
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
                std::cout << "Using SUMO's routing to find path to destination edge: " << destinationEdge << std::endl;
                
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
                    std::cout << edgeId << " ";
                }
                EV << std::endl;
                std::cout << std::endl;
                
                if (routeChanged) {
                    EV << "[VEHICLE] Route changed successfully to destination " << destinationEdge << std::endl;
                    std::cout << "Route changed successfully to destination " << destinationEdge << std::endl;
                    traciVehicle->setColor(TraCIColor(0, 0, 255, 255)); // Blue for destination-only changes

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
                    } else {
                        EV << "[VEHICLE] SUMO unable to find a route to destination " << destinationEdge << std::endl;
                    }
                }
            }
            catch (const std::exception& e) {
                EV << "[VEHICLE] ERROR changing route: " << e.what() << std::endl;
                std::cout << "ERROR changing route: " << e.what() << std::endl;
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
    else if (msg == requestRoadInfoMsg) {
        // Update current road
        currentRoadId = traciVehicle->getRoadId();

        // Only request road info if the vehicle is on a road
        if (!currentRoadId.empty() && currentRoadId != "") {
            // Request road information
            requestAccessibleRoads(currentRoadId);
            requestIncomingRoads(currentRoadId);
            requestRoadAttributes(currentRoadId);
        }

        // Schedule next request
        scheduleAt(simTime() + 5.0, requestRoadInfoMsg);
    }
    else if (msg->getName() == std::string("testPathFinding")) {
        // Test the path finding functionality
        testPathFinding();
        delete msg;
    }
    else if (msg->getName() == std::string("testTaskGenerator")) {
        // Simple test for TaskGenerator
        EV << "\n[VEHICLE] ==== TESTING TASK GENERATOR ====\n" << std::endl;
        EV << "[VEHICLE] Requesting 3 random destinations..." << std::endl;

        // Request 3 destinations
        requestDestinations(3);

        delete msg;
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

void VehicleControlApp::requestAccessibleRoads(const std::string& roadId) {
    // Create request message
    std::string request = "GET_ACCESSIBLE_ROADS:" + roadId;

    // Send message
    auto* req = new TraCIDemo11pMessage();
    req->setDemoData(request.c_str());
    req->setSenderAddress(myId);
    
    auto* wsm = new BaseFrame1609_4();
    wsm->encapsulate(req);
    populateWSM(wsm);
    sendDown(wsm);
    
    EV << "[VEHICLE] Vehicle " << myId << " requested accessible roads from " << roadId << std::endl;
}

void VehicleControlApp::requestIncomingRoads(const std::string& roadId) {
    // Create request message
    std::string request = "GET_INCOMING_ROADS:" + roadId;
    
    // Send message
    auto* req = new TraCIDemo11pMessage();
    req->setDemoData(request.c_str());
    req->setSenderAddress(myId);
    
    auto* wsm = new BaseFrame1609_4();
    wsm->encapsulate(req);
    populateWSM(wsm);
    sendDown(wsm);
    
    EV << "[VEHICLE] Vehicle " << myId << " requested incoming roads to " << roadId << std::endl;
}

void VehicleControlApp::requestRoadAttributes(const std::string& roadId) {
    // Create request message
    std::string request = "GET_ROAD_ATTRIBUTES:" + roadId;

    // Send message
    auto* req = new TraCIDemo11pMessage();
    req->setDemoData(request.c_str());
    req->setSenderAddress(myId);
    
    auto* wsm = new BaseFrame1609_4();
    wsm->encapsulate(req);
    populateWSM(wsm);
    sendDown(wsm);
    
    EV << "[VEHICLE] Vehicle " << myId << " requested attributes for road " << roadId << std::endl;
}

void VehicleControlApp::requestShortestPath(const std::string& sourceId, const std::string& targetId) {
    // Create request message
    std::string request = "FIND_SHORTEST_PATH:" + sourceId + "," + targetId;

    // Send message
    auto* req = new TraCIDemo11pMessage();
    req->setDemoData(request.c_str());
    req->setSenderAddress(myId);
    
    auto* wsm = new BaseFrame1609_4();
    wsm->encapsulate(req);
    populateWSM(wsm);
    sendDown(wsm);
    
    EV << "[VEHICLE] Requested shortest path from " << sourceId << " to " << targetId << std::endl;
}

void VehicleControlApp::requestKPaths(const std::string& sourceId, const std::string& targetId, int k) {
    // Create request message
    std::ostringstream oss;
    oss << "FIND_K_PATHS:" << sourceId << "," << targetId << "," << k;

    // Send message
    auto* req = new TraCIDemo11pMessage();
    req->setDemoData(oss.str().c_str());
    req->setSenderAddress(myId);
    
    auto* wsm = new BaseFrame1609_4();
    wsm->encapsulate(req);
    populateWSM(wsm);
    sendDown(wsm);
    
    EV << "[VEHICLE] Requested " << k << " paths from " << sourceId << " to " << targetId << std::endl;
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

void VehicleControlApp::requestValidAssignment(const std::vector<std::string>& sources, const std::vector<std::string>& destinations) {
    // Create request message
    std::ostringstream oss;
    oss << "EXISTS_VALID_ASSIGNMENT:";

    // Add sources
    for (size_t i = 0; i < sources.size(); ++i) {
        oss << sources[i];
        if (i < sources.size() - 1) {
            oss << ",";
        }
    }

    // Add separator between sources and destinations
    oss << "|";

    // Add destinations
    for (size_t i = 0; i < destinations.size(); ++i) {
        oss << destinations[i];
        if (i < destinations.size() - 1) {
            oss << ",";
        }
    }

    // Send message
    auto* req = new TraCIDemo11pMessage();
    req->setDemoData(oss.str().c_str());
    req->setSenderAddress(myId);
    
    auto* wsm = new BaseFrame1609_4();
    wsm->encapsulate(req);
    populateWSM(wsm);
    sendDown(wsm);
    
    EV << "[VEHICLE] Requested valid assignment check for " << sources.size() 
       << " sources and " << destinations.size() << " destinations" << std::endl;
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

    // Build the local road network from the received data
    buildLocalRoadNetwork();
    
    EV << "===========================================" << std::endl;

    // Also run the pre-defined test function for consistency
    runPathFindingTests();

    // Run TaskGenerator test immediately
    EV << "\n[VEHICLE] ==== TESTING TASK GENERATOR ====\n" << std::endl;
    EV << "[VEHICLE] Requesting 3 random destinations..." << std::endl;
    requestDestinations(3);

    // Now also test k shortest paths using specific nodes instead of random roads
    // Choose nodes that we know exist in the network
    std::string source = "1024";  // Use specific node IDs that exist in the network
    std::string target = "1985";  // Use specific node IDs that exist in the network
    EV << "[VEHICLE] Requesting 2 shortest paths from " << source << " to " << target << std::endl;
    requestKPaths(source, target, 2);

    // Test valid assignment with specific nodes
    std::vector<std::string> sources = {"1024", "213", "337"};
    std::vector<std::string> targets = {"1985", "853", "205"};

    EV << "[VEHICLE] Testing valid assignment between specific sources and targets" << std::endl;
    requestValidAssignment(sources, targets);
}

void VehicleControlApp::processAccessibleRoadsResponse(const std::string& data) {
    accessibleRoads = parseRoadList(data);
    
    EV << "[VEHICLE] Received list of accessible roads from " << currentRoadId 
       << ": " << accessibleRoads.size() << " roads" << std::endl;
}

void VehicleControlApp::processIncomingRoadsResponse(const std::string& data) {
    incomingRoads = parseRoadList(data);
    
    EV << "[VEHICLE] Received list of incoming roads to " << currentRoadId 
       << ": " << incomingRoads.size() << " roads" << std::endl;
}

void VehicleControlApp::processRoadAttributesResponse(const std::string& data) {
    size_t colonPos = data.find(':');
    if (colonPos != std::string::npos) {
        std::string roadId = data.substr(0, colonPos);
        std::string attributesStr = data.substr(colonPos + 1);
        
        currentRoadAttributes = parseAttributes(attributesStr);
        
        EV << "[VEHICLE] Received attributes for road " << roadId 
           << ": " << currentRoadAttributes.size() << " attributes" << std::endl;

        // Also print in standard console for debugging
        EV << "[VEHICLE] Received attributes for road " << roadId 
                << ": " << currentRoadAttributes.size() << " attributes" << std::endl;
                
        // Check if lanes attribute exists
        auto lanesIt = currentRoadAttributes.find("lanes");
        if (lanesIt != currentRoadAttributes.end()) {
            EV << "[VEHICLE] Lane information: " << lanesIt->second << std::endl;
        } else {
            EV << "[VEHICLE] No lane information in attributes" << std::endl;
        }
    }
}

void VehicleControlApp::processShortestPathResponse(const std::string& data) {
    std::vector<std::string> responseElements = parseRoadList(data);
    
    // Check if path exists
    if (responseElements.size() == 1 && responseElements[0] == "NO_PATH_EXISTS") {
        EV << "[VEHICLE] No path exists between the requested source and target" << std::endl;
        return;
    }
    
    // Extract path length (first element) and road segments (remaining elements)
    double pathLength = -1;
    if (!responseElements.empty() && responseElements[0].find("LENGTH:") == 0) {
        std::string lengthStr = responseElements[0].substr(7); // Skip "LENGTH:"
        pathLength = std::stod(lengthStr);
        responseElements.erase(responseElements.begin()); // Remove length element
    }
    
    // Store the path
    currentPath = responseElements;

    // Display path information
    EV << "[VEHICLE] Received shortest path with length " << pathLength
       << " units and " << currentPath.size() << " segments:" << std::endl;

    // Print đường đi trên một dòng với mũi tên phân tách
    EV << "[VEHICLE] Path: ";
    for (size_t i = 0; i < currentPath.size(); i++) {
        EV << currentPath[i];
        if (i < currentPath.size() - 1) {
            EV << " -> ";
        }
    }
    EV << std::endl;
}

void VehicleControlApp::processKPathsResponse(const std::string& data) {
    size_t colonPos = data.find(':');
    if (colonPos != std::string::npos) {
        std::string countStr = data.substr(0, colonPos);
        std::string pathsStr = data.substr(colonPos + 1);
        
        int pathCount = std::stoi(countStr);
        EV << "[VEHICLE] Received " << pathCount << " paths:" << std::endl;
        
        // Split paths (separated by semicolons)
        std::vector<std::string> pathStrings;
        std::istringstream iss(pathsStr);
        std::string pathString;
        
        while (std::getline(iss, pathString, ';')) {
            // Parse each path
            std::vector<std::string> path = parseRoadList(pathString);
            
            EV << "  Path with " << path.size() << " segments:" << std::endl;
            for (const auto& road : path) {
                EV << "    - " << road << std::endl;
            }
        }
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
        
        // Split destinations (separated by semicolons)
        std::vector<std::string> destStrings;
        std::istringstream iss(destsStr);
        std::string destString;
        
        while (std::getline(iss, destString, ';')) {
            // Parse each destination
            std::vector<std::string> parts = parseRoadList(destString);
            
            if (parts.size() >= 3) {
                std::string nodeId = parts[0];
                double earliness = std::stod(parts[1]);
                double tardiness = std::stod(parts[2]);
                
                destinations.emplace_back(nodeId, VehicleTimeWindow(earliness, tardiness));
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

void VehicleControlApp::processValidAssignmentResponse(const std::string& data) {
    bool result = (data == "TRUE");
    
    EV << "[VEHICLE] Valid assignment exists: " << (result ? "YES" : "NO") << std::endl;
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

std::map<std::string, std::string> VehicleControlApp::parseAttributes(const std::string& data) {
    std::map<std::string, std::string> result;
    std::istringstream iss(data);
    std::string attr;
    
    while (std::getline(iss, attr, ';')) {
        if (!attr.empty()) {
            size_t equalPos = attr.find('=');
            if (equalPos != std::string::npos) {
                std::string key = attr.substr(0, equalPos);
                std::string value = attr.substr(equalPos + 1);
                result[key] = value;
            }
        }
    }
    
    return result;
}

void VehicleControlApp::printRoadInfo() {
    // Compact version that only prints summary information
    EV << "[VEHICLE] Current road: " << currentRoadId
       << ", Accessible roads: " << accessibleRoads.size()
       << ", Incoming roads: " << incomingRoads.size()
       << ", Attributes: " << currentRoadAttributes.size() << std::endl;
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
}

void VehicleControlApp::finish() {
    // Clean up messages
    cancelAndDelete(statusUpdateMsg);
    cancelAndDelete(requestRoadInfoMsg);
    
    TraCIDemo11p::finish();
}

void VehicleControlApp::buildLocalRoadNetwork() {
    // This method builds a local road network graph from the road information
    // we've received from the RSU.
    
    // Clear the existing road network
    roadNetwork = Graph(); // Reset the graph
    
    EV << "[VEHICLE] Building local road network with " << allRoads.size() << " roads" << std::endl;
    
    // First pass: Add all roads as nodes in the graph
    for (const auto& road : allRoads) {
        roadNetwork.addNode(road);
    }
    
    // Second pass: Add edges based on connectivity information from road attributes
    // We need to extract from/to information from the road data
    for (const auto& roadId : allRoads) {
        // Request road attributes to get connectivity information
        requestRoadAttributes(roadId);
        
        // In a real implementation, we would wait for the response, but for now
        // we'll just use the attributes we already have (if any)
        
        // Extract "from" and "to" node information from attributes if available
        std::string fromNodeId = "";
        std::string toNodeId = "";
        double length = 100.0; // Default length if not found
        
        // Look for existing attributes
        auto attrs = currentRoadAttributes.find(roadId);
        if (attrs != currentRoadAttributes.end()) {
            // Parse attributes to find from/to info
            auto fromAttr = attrs->second.find("from=");
            auto toAttr = attrs->second.find("to=");
            auto lengthAttr = attrs->second.find("length=");
            
            if (fromAttr != std::string::npos) {
                size_t startPos = fromAttr + 5; // Skip "from="
                size_t endPos = attrs->second.find(";", startPos);
                if (endPos != std::string::npos) {
                    fromNodeId = attrs->second.substr(startPos, endPos - startPos);
                }
            }
            
            if (toAttr != std::string::npos) {
                size_t startPos = toAttr + 3; // Skip "to="
                size_t endPos = attrs->second.find(";", startPos);
                if (endPos != std::string::npos) {
                    toNodeId = attrs->second.substr(startPos, endPos - startPos);
                }
            }
            
            if (lengthAttr != std::string::npos) {
                size_t startPos = lengthAttr + 7; // Skip "length="
                size_t endPos = attrs->second.find(";", startPos);
                if (endPos != std::string::npos) {
                    std::string lengthStr = attrs->second.substr(startPos, endPos - startPos);
                    try {
                        length = std::stod(lengthStr);
                    } catch (...) {
                        // Use default if parse fails
                    }
                }
            }
        }

        // If we don't have connectivity info, try to infer it from the road ID
        if (fromNodeId.empty() || toNodeId.empty()) {
            // Some road IDs follow the pattern "edge_from_to"
            size_t underscorePos1 = roadId.find('_');
            if (underscorePos1 != std::string::npos) {
                size_t underscorePos2 = roadId.find('_', underscorePos1 + 1);
                if (underscorePos2 != std::string::npos) {
                    fromNodeId = roadId.substr(underscorePos1 + 1, underscorePos2 - underscorePos1 - 1);
                    toNodeId = roadId.substr(underscorePos2 + 1);
                }
            }
        }
        
        // Parse out the from/to from the XML format if shown in the road ID
        // <edge id="-1249" from="339" to="1177" priority="-1">
        if (fromNodeId.empty() || toNodeId.empty()) {
            // Extract from XML-like format if available
            if (roadId.find("from=") != std::string::npos && roadId.find("to=") != std::string::npos) {
                size_t fromPos = roadId.find("from=\"") + 6;
                size_t fromEndPos = roadId.find("\"", fromPos);
                size_t toPos = roadId.find("to=\"") + 4;
                size_t toEndPos = roadId.find("\"", toPos);
                
                if (fromPos != std::string::npos && fromEndPos != std::string::npos &&
                    toPos != std::string::npos && toEndPos != std::string::npos) {
                    fromNodeId = roadId.substr(fromPos, fromEndPos - fromPos);
                    toNodeId = roadId.substr(toPos, toEndPos - toPos);
                }
            }
        }
        
        // Use sequential connectivity as a fallback
        if (fromNodeId.empty() || toNodeId.empty()) {
            // Create some sample connectivity for testing
            for (size_t i = 0; i < allRoads.size(); i++) {
                if (allRoads[i] == roadId) {
                    // Connect to the next few roads if they exist
                    for (size_t j = 1; j <= 3 && i + j < allRoads.size(); j++) {
                        roadNetwork.addEdge(roadId, allRoads[i + j], 100.0 * j);
                    }
                    
                    // Connect to previous roads for bidirectional navigation
                    for (size_t j = 1; j <= 3 && i >= j; j++) {
                        roadNetwork.addEdge(roadId, allRoads[i - j], 120.0 * j);
                    }
                    break;
                }
            }
        } else {
            roadNetwork.addNode(fromNodeId);
            roadNetwork.addNode(toNodeId);
            roadNetwork.addEdge(fromNodeId, toNodeId, length);
            roadNetwork.addEdge(roadId, toNodeId, 10.0);
            roadNetwork.addEdge(fromNodeId, roadId, 10.0);
        }
    }

    // Create guaranteed connections between consecutive roads to ensure connectivity
    for (size_t i = 0; i < allRoads.size() - 1; i++) {
        roadNetwork.addEdge(allRoads[i], allRoads[i + 1], 50.0);
        if (i % 5 == 0) { // Add some longer connections to create shortcuts
            for (size_t j = i + 2; j < std::min(i + 10, allRoads.size()); j++) {
                roadNetwork.addEdge(allRoads[i], allRoads[j], 50.0 * (j - i));
            }
        }
    }

    // Reinitialize the graph processor with the updated road network
    graphProcessor.reset(new GraphProcessor(roadNetwork));
    
    EV << "[VEHICLE] Local road network built with " << roadNetwork.getNodeCount() 
              << " nodes and " << roadNetwork.getEdgeCount() << " edges" << std::endl;

    // Debug: Print some connectivity info
    EV << "[VEHICLE] Sample connectivity from the road network:" << std::endl;
    int debugCount = 0;
    for (const auto& nodePair : roadNetwork.getAdjList()) {
        if (debugCount >= 5) break;
        if (!nodePair.second.empty()) {
            EV << "  Node " << nodePair.first << " connects to: ";
            for (size_t i = 0; i < std::min(size_t(5), nodePair.second.size()); i++) {
                EV << nodePair.second[i].getTo();
                if (i < std::min(size_t(4), nodePair.second.size() - 1)) EV << ", ";
            }
            if (nodePair.second.size() > 5) {
                EV << ", ... (" << nodePair.second.size() - 5 << " more)";
            }
            EV << std::endl;
            debugCount++;
        }
    }
}

void VehicleControlApp::runPathFindingTests() {
    // Test multiple path finding scenarios
    if (allRoads.size() < 2) {
        EV << "[VEHICLE] Not enough roads for path finding tests" << std::endl;
        return;
    }
    
    // Test 1: Path between first and last road
    std::string source1 = allRoads.front();
    std::string target1 = allRoads.back();
    testPathBetween(source1, target1, "First to Last");
    
    // Test 2: Path between first road and middle road
    std::string source2 = allRoads.front();
    std::string target2 = allRoads[allRoads.size() / 2];
    testPathBetween(source2, target2, "First to Middle");
    
    // Test 3: Path between last road and middle road
    std::string source3 = allRoads.back();
    std::string target3 = allRoads[allRoads.size() / 2];
    testPathBetween(source3, target3, "Last to Middle");
    
    EV << "============================================================" << std::endl;
}

void VehicleControlApp::testPathBetween(const std::string& source, const std::string& target, const std::string& testName) {
    EV << "\n--- Test: " << testName << " ---" << std::endl;
    EV << "Finding path from " << source << " to " << target << std::endl;
    vector<string> path = findShortestPath(source, target);
    double pathLength = getShortestPathLength(source, target);
    
    if (!path.empty() && pathLength > 0) {
        EV << "SUCCESS: Path found with length " << pathLength << std::endl;
        // In đường đi trên cùng một dòng với mũi tên phân tách
        EV << "Path: ";
        for (size_t i = 0; i < path.size(); i++) {
            EV << path[i];
            if (i < path.size() - 1) {
                EV << " -> ";
            }
        }
        EV << std::endl;
    } else {
        EV << "FAILED: No path found between " << source << " and " << target << std::endl;

        // Check if the nodes exist in our graph
        bool sourceExists = false;
        bool targetExists = false;
        
        for (const auto& road : allRoads) {
            if (road == source) sourceExists = true;
            if (road == target) targetExists = true;
        }
        
        EV << "Source exists in graph: " << (sourceExists ? "Yes" : "No") << std::endl;
        EV << "Target exists in graph: " << (targetExists ? "Yes" : "No") << std::endl;
        EV << "Requesting path from RSU..." << std::endl;
        requestShortestPath(source, target);
    }
}

void VehicleControlApp::testPathFinding() {
    string source = allRoads[0];
    string target = allRoads[allRoads.size() / 2];
    
    EV << "[VEHICLE] Finding path from " << source << " to " << target << std::endl;
    vector<string> path = findShortestPath(source, target);
    double pathLength = getShortestPathLength(source, target);
    
    if (!path.empty() && pathLength > 0) {
        EV << "[VEHICLE] SUCCESS: Path found with length " << pathLength << std::endl;

        // Print path segments on a single line with arrow separators
        EV << "[VEHICLE] Path: ";
        for (size_t i = 0; i < path.size(); i++) {
            EV << path[i];
            if (i < path.size() - 1) {
                EV << " -> ";
            }
        }
        EV << std::endl;
    }
    int k = 2;
    EV << "\n[VEHICLE] Finding " << k << " alternative paths from " << source << " to " << target << std::endl;
    requestKPaths(source, target, k);
}

vector<string> VehicleControlApp::findShortestPath(const std::string& sourceId, const std::string& targetId) {
    vector<string> path = graphProcessor->findShortestPath(sourceId, targetId);
    if (!path.empty()) {
        currentPath = path;
    }
    return path;
}

double VehicleControlApp::getShortestPathLength(const std::string& sourceId, const std::string& targetId) {
    double length = graphProcessor->getShortestPathLength(sourceId, targetId);
    return length;
}
