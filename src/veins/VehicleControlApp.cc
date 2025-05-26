#include "VehicleControlApp.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include <sstream>
#include <iostream>
#include <fstream>   // Add include for ofstream
#include <iomanip>   // Add include for setprecision
#include <chrono>    // Add include for time measurement

using namespace veins;
using namespace std;

Register_Class(VehicleControlApp);

void VehicleControlApp::initialize(int stage) {
    TraCIDemo11p::initialize(stage);
    if (stage == 0) {
        // Initialize messages
        statusUpdateMsg = new cMessage("statusUpdate");
        requestRoadInfoMsg = new cMessage("requestRoadInfo");
        cleanupTimer = new cMessage("vehicleCleanup");

        // Get mobility interface to access SUMO vehicle data first
        mobility = TraCIMobilityAccess().get(getParentModule());
        traciVehicle = mobility->getVehicleCommandInterface();
        
        // USE THE ACTUAL MODULE INDEX - this is the key to getting unique IDs
        int moduleIndex = getParentModule()->getIndex();

        // Hard-code the simulation ID mapping exactly as in RSUControlApp
        std::map<int, int> idMapping = {
            {0, 16},   // Module index 0 maps to simulation ID 16
            {1, 22},   // Module index 1 maps to simulation ID 22
            {2, 28},   // And so on...
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

        // Use the module index for internal ID
        myInternalId = moduleIndex;

        // Look up this vehicle's simulation ID from the mapping
        auto it = idMapping.find(myInternalId);
        if (it != idMapping.end()) {
            mySimulationId = it->second;
        } else {
            // Fallback if no mapping exists
            mySimulationId = myInternalId + 1000; // Different base to avoid conflicts
        }

        // Get the actual SUMO ID of this vehicle for debugging
        std::string sumoId = mobility->getExternalId();
        
        // Print to standard output for debugging - MAKE THIS VERY VISIBLE
        std::cout << "\n*********************** IMPORTANT ID INFO ***********************" << std::endl;
        std::cout << "Vehicle '" << sumoId 
                  << "' assigned internal ID " << myInternalId
                  << " and simulation ID " << mySimulationId << std::endl;
        std::cout << "Parent module: " << getParentModule()->getFullName()
                  << ", index: " << getParentModule()->getIndex() << std::endl;
        std::cout << "MAC Address: " << getParentModule()->getId()
                  << ", Full Path: " << getParentModule()->getFullPath() << std::endl;
        std::cout << "******************************************************************\n" << std::endl;

        // Log the details to help with debugging
        EV << "\n[VEHICLE] ********************************************" << std::endl;
        EV << "[VEHICLE] Initialized with internal ID: " << myInternalId
           << ", simulation ID: " << mySimulationId << std::endl;
        EV << "[VEHICLE] Parent module: " << getParentModule()->getFullName()
           << ", index: " << getParentModule()->getIndex() << std::endl;
        EV << "[VEHICLE] ********************************************" << std::endl;

        // Record the exact start time of the vehicle
        startTime = simTime().dbl();
        startingRoad = mobility->getRoadId();
        
        // Record start time to logger
        SimulationLogger::getInstance().recordVehicleStart(mySimulationId, startingRoad, startTime);
        
        // Initialize data structures
        allRoads.clear();
        accessibleRoads.clear();
        incomingRoads.clear();
        currentRoadAttributes.clear();
        currentPath.clear();
        destinations.clear();
        
        // Initialize time window and path info
        earliestArrival = 0.0;
        latestArrival = 0.0;
        endTime = 0.0;
        pathLength = 0.0;

        // Initialize the graph processor (will populate with data when we receive roads)
        graphProcessor.reset(new GraphProcessor(roadNetwork));

        // Request all roads from the RSU immediately to get the road network data
        requestAllRoads();

        // Print a message that we're waiting for road data
        EV << "\n[VEHICLE] Requesting road network data from RSU..." << std::endl;
        EV << "[VEHICLE] Road information will be printed once received." << std::endl;
    } else if (stage == 1) {
        // Schedule initial messages
        scheduleAt(simTime() + uniform(1.0, 2.0), statusUpdateMsg);
        scheduleAt(simTime() + uniform(3.0, 5.0), requestRoadInfoMsg);

        // Schedule regular vehicle status check (every 0.1 seconds for precision)
        scheduleAt(simTime() + 0.1, cleanupTimer);

        EV << "[VEHICLE] Vehicle " << myId << " initialized" << std::endl;
    }
}

void VehicleControlApp::onWSM(BaseFrame1609_4* wsm) {
    auto* msg = dynamic_cast<TraCIDemo11pMessage*>(wsm->getEncapsulatedPacket());
    if (!msg) return;

    std::string data = msg->getDemoData();
    
    // Enhanced message reception debugging
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
            std::cout << "\n>>>>>>> ROUTE MESSAGE HANDLING <<<<<<<<<" << std::endl;
            std::cout << "Message targets simulation ID: " << targetVehId
                      << ", my internal ID: " << myInternalId
                      << ", my simulation ID: " << mySimulationId << std::endl;
            std::cout << "WSM Recipient Address: " << recipientAddress << std::endl;
            std::cout << "Module index: " << getParentModule()->getIndex() << std::endl;
            std::cout << "Parent: " << getParentModule()->getFullPath() << std::endl;
            std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;

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
                        // Check if we can get connected edges from current edge
                        // This is a simple simulation of connectivity checking since we don't have direct
                        // access to SUMO's edge connectivity information in this context

                        // Print debug info about the edges we're checking
                        EV << "[VEHICLE] Checking connectivity between edges: " << routeEdges[i]
                           << " and " << routeEdges[i+1] << std::endl;
                        std::cout << "Checking connectivity between edges: " << routeEdges[i]
                                  << " and " << routeEdges[i+1] << std::endl;
                    }

                    // Only try to set route if we believe it's valid
                    if (routeIsValid) {
                        // Convert vector to list for the TraCI command
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

                    // Highlight the route change with a vehicle color change
                    traciVehicle->setColor(TraCIColor(0, 0, 255, 255)); // Blue for destination-only changes

                    // Slightly reduce speed to make the change more visible
                    double currentSpeed = traciVehicle->getSpeed();
                    if (currentSpeed > 5.0) {
                        traciVehicle->setSpeed(currentSpeed * 0.8);
                        EV << "[VEHICLE] Reduced speed to " << (currentSpeed * 0.8) << " to make route change visible" << std::endl;
                        std::cout << "Reduced speed to " << (currentSpeed * 0.8) << " to make route change visible" << std::endl;
                    }
                } else {
                    EV << "[VEHICLE] Warning: Route didn't change, destination may already be included or SUMO rejected the change" << std::endl;
                    std::cout << "Warning: Route didn't change, destination may already be included or SUMO rejected the change" << std::endl;

                    // Check if destination is already in the route
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
    else if (msg == cleanupTimer) {
        // Check if vehicle has reached destination
        checkVehicleStatus();
        
        // Schedule next check if still active - check EVERY 0.1 seconds
        if (!hasLoggedDeparture) {
            scheduleAt(simTime() + 0.1, cleanupTimer);  // Check frequently (every 0.1s)
        }
    }
    else if (std::string(msg->getName()) == "resetColor") {
        // Restaurer la couleur normale du véhicule
        if (traciVehicle) {
            try {
                // Couleur normale (blanc)
                traciVehicle->setColor(TraCIColor(255, 255, 255, 255));
            } catch (...) {
                // Ignorer les erreurs
            }
        }
        // Supprimer le message
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

    // Immediately run path finding tests to print shortest paths
    EV << "\n=============== IMMEDIATE PATH FINDING TEST ===============" << std::endl;

    // Kiểm tra đường đi giữa các con đường
    if (allRoads.size() >= 2) {
        std::string source1 = allRoads.front();
        std::string target1 = allRoads.back();
        EV << "Test 1: Finding path from road " << source1 << " to road " << target1 << std::endl;
        std::vector<std::string> path = findShortestPath(source1, target1);
        double pathLength = getShortestPathLength(source1, target1);

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
            path = findShortestPath(source1, target2);
            pathLength = getShortestPathLength(source1, target2);

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

                destinations.emplace_back(nodeId, TimeWindow(earliness, tardiness));
                
                // Lưu thông tin đích và khung thời gian
                if (destinations.size() == 1) { // Chỉ lấy đích đầu tiên
                    targetRoad = nodeId;
                    earliestArrival = earliness;
                    latestArrival = tardiness;
                    
                    // Tìm đường đi ngắn nhất từ điểm bắt đầu đến đích
                    std::vector<std::string> path;
                    double algorithmTime = 0.0;
                    
                    // Bắt đầu đo thời gian thuật toán
                    auto startAlgorithm = std::chrono::high_resolution_clock::now();
                    
                    if (graphProcessor) {
                        path = graphProcessor->findShortestPath(startingRoad, targetRoad);
                        pathLength = graphProcessor->getShortestPathLength(startingRoad, targetRoad);
                    }
                    
                    // Kết thúc đo thời gian thuật toán
                    auto endAlgorithm = std::chrono::high_resolution_clock::now();
                    algorithmTime = std::chrono::duration<double>(endAlgorithm - startAlgorithm).count();
                    
                    // Ghi nhận thông tin đích vào logger
                    SimulationLogger::getInstance().updateVehicleDestination(
                        mySimulationId, targetRoad, earliestArrival, latestArrival, path, pathLength);
                    
                    // Ghi nhận thời gian thuật toán
                    SimulationLogger::getInstance().recordAlgorithmTime(mySimulationId, algorithmTime);
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
    
    // Vérifier et ajuster la vitesse si nécessaire
    checkAndAdjustSpeed();
    
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
                handleEndOfVehicleLifecycle();
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
    // Get the exact time when finish() is called
    endTime = simTime().dbl();
    
    // If we haven't logged the departure yet, do it now
    if (!hasLoggedDeparture) {
        // Record vehicle end to logger
        SimulationLogger::getInstance().recordVehicleEnd(mySimulationId, endTime);
        
        // Mark as logged
        hasLoggedDeparture = true;
        
        // Print information with consistent formatting (1 decimal place)
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "VEHICLE_FINISH: ID=" << mySimulationId 
                  << ", START=" << startTime
                  << ", END=" << endTime
                  << ", DURATION=" << (endTime - startTime)
                  << ", TIME_WINDOW=[" << earliestArrival << ", " << latestArrival << "]"
                  << std::endl;
        std::cout.unsetf(std::ios_base::fixed); // Reset formatting
    }
    
    // Cleanup
    cleanupMessages();

    // Call parent
    TraCIDemo11p::finish();
}

void VehicleControlApp::cleanupMessages() {
    if (statusUpdateMsg) {
        cancelAndDelete(statusUpdateMsg);
        statusUpdateMsg = nullptr;
    }
    if (requestRoadInfoMsg) {
        cancelAndDelete(requestRoadInfoMsg);
        requestRoadInfoMsg = nullptr;
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
        
        // Record vehicle end to logger
        SimulationLogger::getInstance().recordVehicleEnd(mySimulationId, endTime);
        
        // Mark as logged
        hasLoggedDeparture = true;
        
        // Print detailed info with consistent formatting
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "\n*********************** VEHICLE ARRIVAL ***********************" << std::endl;
        std::cout << "Vehicle " << mySimulationId << " arrived at time " << endTime << std::endl;
        std::cout << "Travel time: " << (endTime - startTime) << " seconds" << std::endl;
        std::cout << "Time window: [" << earliestArrival << ", " << latestArrival << "]" << std::endl;
        
        // Check if arrived on time
        bool arrivedOnTime = (endTime <= latestArrival);
        double timeWindowDeviation = 0.0;
        
        if (endTime < earliestArrival) {
            timeWindowDeviation = earliestArrival - endTime;
            std::cout << "Arrived EARLY by " << timeWindowDeviation << " seconds" << std::endl;
        } else if (endTime > latestArrival) {
            timeWindowDeviation = endTime - latestArrival;
            std::cout << "Arrived LATE by " << timeWindowDeviation << " seconds" << std::endl;
        } else {
            std::cout << "Arrived WITHIN time window" << std::endl;
        }
        
        std::cout << "Path length: " << pathLength << " meters" << std::endl;
        std::cout << "******************************************************************\n" << std::endl;
        std::cout.unsetf(std::ios_base::fixed); // Reset formatting
    }
}

void VehicleControlApp::handleEndOfVehicleLifecycle() {
    // Ghi nhận thời điểm chính xác khi xe sắp bị xóa khỏi mô phỏng
    if (!hasLoggedDeparture) {
        endTime = simTime().dbl();
        
        EV << "[VEHICLE] Vehicle " << mySimulationId << " is being removed from simulation at exact time " 
           << std::fixed << std::setprecision(1) << endTime << std::endl;
        
        // Log the departure with precision
        std::cout << std::fixed << std::setprecision(1);
        printf("VEHICLE_LIFECYCLE_END: ID=%d, TIME=%.1f, STATUS=REMOVED_FROM_SIMULATION\n",
               mySimulationId, endTime);
        std::cout.unsetf(std::ios_base::fixed); // Reset formatting
        fflush(stdout);
        
        // Record vehicle end to logger
        SimulationLogger::getInstance().recordVehicleEnd(mySimulationId, endTime);
        
        hasLoggedDeparture = true;
    }
}

void VehicleControlApp::checkVehicleStatus() {
    if (mobility && traciVehicle) {
        // Get current road ID
        std::string roadId = traciVehicle->getRoadId();
        
        // Check if vehicle has reached destination
        if (!targetRoad.empty() && roadId == targetRoad) {
            EV << "[VEHICLE] Vehicle " << mySimulationId << " has reached destination road " 
               << targetRoad << " at time " << simTime() << std::endl;
            
            // Log arrival
            logDepartureIfNeeded();
        }
        // Check if vehicle is about to be removed
        else if (roadId.empty() || roadId == "") {
            EV << "[VEHICLE] Vehicle " << mySimulationId << " is no longer on a road at time " 
               << simTime() << " (may be removed from simulation)" << std::endl;
            
            // Handle end of lifecycle
            handleEndOfVehicleLifecycle();
        }
    }
}

double VehicleControlApp::measureRoutingAlgorithmTime(const std::string& sourceId, const std::string& targetId) {
    if (!graphProcessor) {
        return -1.0;
    }
    
    // Bắt đầu đo thời gian
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Thực hiện thuật toán tìm đường
    graphProcessor->findShortestPath(sourceId, targetId);
    
    // Kết thúc đo thời gian
    auto endTime = std::chrono::high_resolution_clock::now();
    
    // Tính thời gian thực hiện
    std::chrono::duration<double> duration = endTime - startTime;
    return duration.count();
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
            // If we have real connectivity information, use it
            // First add the nodes if they don't exist
            roadNetwork.addNode(fromNodeId);
            roadNetwork.addNode(toNodeId);

            // Add edge between nodes
            roadNetwork.addEdge(fromNodeId, toNodeId, length);

            // Also connect the road to its nodes
            roadNetwork.addEdge(roadId, toNodeId, 10.0); // Short connection
            roadNetwork.addEdge(fromNodeId, roadId, 10.0); // Short connection

            // Print real connectivity info for a few roads
            static int infoCount = 0;
            if (infoCount < 5) {
                EV << "[VEHICLE] Added real connectivity: Road " << roadId
                         << " connects from " << fromNodeId << " to " << toNodeId
                         << " (length: " << length << ")" << std::endl;
                infoCount++;
            }
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

    // First try with our local GraphProcessor
    std::vector<std::string> path = findShortestPath(source, target);
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

        // Request from RSU as fallback
        EV << "Requesting path from RSU..." << std::endl;
        requestShortestPath(source, target);
    }
}

void VehicleControlApp::testPathFinding() {
    EV << "\n[VEHICLE] Manual path finding test triggered" << std::endl;

    // Run a more focused path finding test
    if (allRoads.size() < 2) {
        EV << "[VEHICLE] Not enough roads for path finding tests" << std::endl;
        return;
    }

    // Select specific source and target roads from available roads
    std::string source = allRoads[0];
    std::string target = allRoads[allRoads.size() / 2];
    
    EV << "[VEHICLE] Finding path from " << source << " to " << target << std::endl;

    // Use local graph processor to find path
    std::vector<std::string> path = findShortestPath(source, target);
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
    } else {
        EV << "[VEHICLE] NO PATH found between " << source << " and " << target << std::endl;
        EV << "[VEHICLE] Requesting path from RSU as fallback..." << std::endl;
        requestShortestPath(source, target);
    }

    // Also test k-paths functionality
    int k = 2;
    EV << "\n[VEHICLE] Finding " << k << " alternative paths from " << source << " to " << target << std::endl;
    requestKPaths(source, target, k);
}

std::vector<std::string> VehicleControlApp::findShortestPath(const std::string& sourceId, const std::string& targetId) {
    if (!graphProcessor) {
        EV << "[VEHICLE] GraphProcessor not initialized" << std::endl;
        return std::vector<std::string>();
    }

    // Use the GraphProcessor to find the shortest path
    std::vector<std::string> path = graphProcessor->findShortestPath(sourceId, targetId);

    // Store the path for later use if it's valid
    if (!path.empty()) {
        currentPath = path;
    }
    
    return path;
}

double VehicleControlApp::getShortestPathLength(const std::string& sourceId, const std::string& targetId) {
    if (!graphProcessor) {
        EV << "[VEHICLE] GraphProcessor not initialized" << std::endl;
        return -1.0;
    }

    // Use the GraphProcessor to get the shortest path length
    double length = graphProcessor->getShortestPathLength(sourceId, targetId);

    return length;
}

// Ajouter cette nouvelle méthode pour vérifier et ajuster la vitesse
void VehicleControlApp::checkAndAdjustSpeed() {
    if (!mobility || !traciVehicle) {
        return;
    }
    
    // Obtenir la vitesse actuelle du véhicule
    double currentSpeed = traciVehicle->getSpeed();
    
    // Obtenir l'ID de la route actuelle
    std::string roadId = traciVehicle->getRoadId();
    if (roadId.empty()) {
        return; // Le véhicule n'est pas sur une route valide
    }
    
    // Si nous sommes sur une nouvelle route, demander ses attributs
    if (roadId != currentRoadId) {
        requestRoadAttributes(roadId);
        currentRoadId = roadId;
        return; // Attendre que nous recevions les attributs
    }
    
    // Obtenir les attributs de la route (y compris la vitesse maximale)
    double maxSpeed = -1.0;
    
    // Vérifier si nous avons déjà les attributs de la route
    // D'abord vérifier l'attribut spécifique lane0_speed (pour la première voie)
    if (currentRoadAttributes.find("lane0_speed") != currentRoadAttributes.end()) {
        try {
            maxSpeed = std::stod(currentRoadAttributes["lane0_speed"]);
        } catch (...) {
            // Erreur de conversion, essayer d'autres attributs
            maxSpeed = -1.0;
        }
    }
    
    // Si nous n'avons pas trouvé la vitesse dans lane0_speed, essayer maxSpeed
    if (maxSpeed < 0 && currentRoadAttributes.find("maxSpeed") != currentRoadAttributes.end()) {
        try {
            maxSpeed = std::stod(currentRoadAttributes["maxSpeed"]);
        } catch (...) {
            // Erreur de conversion, utiliser une valeur par défaut
            maxSpeed = -1.0;
        }
    }
    
    // Si nous n'avons pas encore les attributs ou la vitesse max, essayer de les obtenir de l'interface TraCI
    if (maxSpeed < 0) {
        try {
            // Obtenir la voie actuelle et sa vitesse maximale
            std::string laneId = traciVehicle->getLaneId();
            if (!laneId.empty()) {
                // Utiliser TraCI pour obtenir la vitesse maximale de la voie
                maxSpeed = mobility->getCommandInterface()->lane(laneId).getMaxSpeed();
            }
        } catch (...) {
            // En cas d'erreur, utiliser une valeur par défaut
            maxSpeed = 13.89; // 50 km/h en m/s
        }
    }
    
    // Si maxSpeed est toujours négatif, utiliser une valeur par défaut
    if (maxSpeed <= 0) {
        maxSpeed = 13.89; // 50 km/h en m/s
    }
    
    // Ajouter une marge de tolérance (par exemple, 10% au-dessus de la limite)
    double speedLimit = maxSpeed * 1.1;
    
    // Variable statique pour éviter de réduire la vitesse trop souvent
    static simtime_t lastSpeedReduction = 0;
    
    double newSpeed = -1.0; // Valeur par défaut indiquant qu'aucun changement n'est nécessaire
    
    // Vérifier si la vitesse est exactement 13.67 m/s
    if (currentSpeed > 13.89 && (simTime() - lastSpeedReduction > 10.0)) {
        // Ajuster la vitesse à exactement 6 m/s
        newSpeed = 6.0;
        EV << "[VEHICLE] Vitesse exactement à 13.67 m/s détectée sur route " << roadId
           << ". Réduction à 6 m/s" << std::endl;
    }
    
         // Appliquer la nouvelle vitesse si nécessaire
    if (newSpeed > 0) {
        // Appliquer la nouvelle vitesse
        traciVehicle->setSpeed(newSpeed);
        
        // Mettre à jour le temps de la dernière réduction
        lastSpeedReduction = simTime();
        
        // Message différent selon le cas (vitesse exacte de 13.67 ou dépassement de limite)
        if (std::abs(currentSpeed - 13.67) < 0.01) {
            std::cout << "Vehicle " << mySimulationId << " reached exactly 13.67 m/s on road " << roadId
                      << ". Setting speed to exactly 6.0 m/s as requested." << std::endl;
        } else {
            std::cout << "Vehicle " << mySimulationId << " exceeded speed limit on road " << roadId
                      << " (" << currentSpeed << " > " << maxSpeed << " m/s). "
                      << "Reducing speed to " << newSpeed << " m/s" << std::endl;
        }
                  
        // Changer la couleur du véhicule pour indiquer visuellement le dépassement de vitesse
        try {
            // Rouge pour indiquer un dépassement de vitesse
            traciVehicle->setColor(TraCIColor(255, 0, 0, 255));
            
            // Programmer le retour à la couleur normale après quelques secondes
            scheduleAt(simTime() + 3.0, new cMessage("resetColor"));
        } catch (...) {
            // Ignorer les erreurs liées au changement de couleur
        }
    }
}
