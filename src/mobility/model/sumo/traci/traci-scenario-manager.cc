#include <fstream>
#include <vector>
#include <set>
#include <algorithm>
#include <stdexcept>
#include <cassert>
#include <sstream>
#include <iostream>
#include <fstream>

#include "./traci-connection.h"
#include "./traci-scenario-manager.h"
#include "./traci-command-interface.h"
#include "./traci-constants.h"
#include "simcore/simulator.h"
#include "simcore/ptr.h"
#include "math/random/rng-manager.h"
#include "io/parameters.h"

#include "basic-models/mobility/rapidxml/rapidxml_utils.hpp"

#define CMD_FILE_SEND 0x75

using namespace std;
#define MYDEBUG std::cout
//#define opp_error opp_opp_error

TraCIScenarioManager* TraCIScenarioManager::inst_ = 0;


void TraCIScenarioManager::Init(const SUMOInputParameters& p) {
  TraCIScenarioManager::inst_ = new TraCIScenarioManager();
  inst_->initialize(p);
}

void TraCIScenarioManager::DeleteInstance()
{
    delete inst_;
    inst_ = 0;
}


TraCIScenarioManager::TraCIScenarioManager() :
		connection(0)
{
	auto seed = RNGMgr().GetSeed(RNGSeed_e::RS_UE_DROP);
	randomStream.SetSeed(seed);
}

TraCIScenarioManager::~TraCIScenarioManager() {
	delete commandIfc;
	delete connection;
}

void TraCIScenarioManager::connectAndStartTrigger() {	
	connection = TraCIConnection::connect(host.c_str(), port);
	commandIfc = new TraCICommandInterface(*connection);
	init_traci();
	return;
}


void TraCIScenarioManager::initialize(const SUMOInputParameters& p) {
	launchConfig = p.launchConfig;
	seed = p.seed;
	connectAt = p.connectAt;
	firstStepAt = p.firstStepAt ;
	updateInterval = p.updateInterval;
	currentat = 0.0;
	penetrationRate = p.penetrationRate;
	if (firstStepAt == -1) firstStepAt = connectAt + updateInterval;
	host = p.host;
	port = p.port;

	sumoparsertype = p.sumoparsertype;		//added new;
	filename = p.filename;

	std::string roiRoads_s = p.roiRoads;
	std::string roiRects_s = p.roiRects;

	vehicleNameCounter = 0;
	vehicleRngIndex = p.vehicleRngIndex;
	numVehicles = 2;
	autoShutdown = p.autoShutdown;
	//mobRng = getRNG(vehicleRngIndex);	//ADD

	// parse roiRoads
	roiRoads.clear();
	std::istringstream roiRoads_i(roiRoads_s);
	std::string road;
	bool start_ = true;
	if(start_){
		char c; roiRoads_i >> c;
		start_ = false;
	}
	while (std::getline(roiRoads_i, road, '&')) {
		roiRoads.push_back(road);
	}

	// parse roiRects
	roiRects.clear();
	std::istringstream roiRects_i(roiRects_s);
	std::string rect;
	bool start = true;
	while (std::getline(roiRects_i, rect, '-')) {
		std::istringstream rect_i(rect);
		if (start){
			char c; rect_i >> c;
			start = false;
		}
		double x1; rect_i >> x1; ASSERT(rect_i);
		char c1; rect_i >> c1; ASSERT(rect_i);
		double y1; rect_i >> y1; ASSERT(rect_i);
		char c2; rect_i >> c2; ASSERT(rect_i);
		double x2; rect_i >> x2; ASSERT(rect_i);
		char c3; rect_i >> c3; ASSERT(rect_i);
		double y2; rect_i >> y2; ASSERT(rect_i);
		roiRects.push_back(std::pair<TraCICoord, TraCICoord>(TraCICoord(x1, y1), TraCICoord(x2, y2)));
	}
	std::cout<<"roiRoads: "<<roiRoads.size()<<"  "<<roiRoads.front()<<std::endl;
	std::cout<<"roiRects: "<<roiRects.size()<<std::endl;
	subscribedVehicles.clear();
	activeVehicleCount = 0;
	parkingVehicleCount = 0;
	drivingVehicleCount = 0;
	autoShutdownTriggered = false;

	std::cout << "initialized TraCIScenarioManager" << std::endl;
}

void TraCIScenarioManager::init_traci() {
	{
		std::pair<uint32_t, std::string> version = std::pair<uint32_t, std::string>(1,"1");//getCommandInterface()->getVersion();
		uint32_t apiVersion = version.first;
		std::string serverVersion = version.second;

		if (apiVersion == 1) {
			//MYDEBUG << "TraCI server \"" << serverVersion << "\" reports API version " << apiVersion << endl;
		}
		else {
			opp_error("TraCI server \"%s\" reports API version %d, which is unsupported. We recommend using the version of sumo-launchd that ships with Veins.", serverVersion.c_str(), apiVersion);
		}
	}
	using namespace rapidxml;
	std::string filename = launchConfig;
	rapidxml::file<char> f(filename.c_str());
	std::string contents(f.data());

	TraCIBuffer buf;
	buf << std::string("sumo-launchd.launch.xml") << contents;
	connection->sendMessage(makeTraCICommand(CMD_FILE_SEND, buf));

	TraCIBuffer obuf(connection->receiveMessage());
	uint8_t cmdLength; obuf >> cmdLength;
	uint8_t commandResp; obuf >> commandResp; if (commandResp != CMD_FILE_SEND) opp_error("Expected response to command %d, but got one for command %d", CMD_FILE_SEND, commandResp);
	uint8_t result; obuf >> result;
	std::string description; obuf >> description;
	if (result != RTYPE_OK) {
		//EV << "Warning: Received non-OK response from TraCI server to command " << CMD_FILE_SEND << ":" << description.c_str() << std::endl;
	}

	{
		std::pair<uint32_t, std::string> version = getCommandInterface()->getVersion();
		uint32_t apiVersion = version.first;
		std::string serverVersion = version.second;

		// if (apiVersion == 8) {
		// 	MYDEBUG << "TraCI server \"" << serverVersion << "\" reports API version " << apiVersion << endl;
		// }
		// else {
		// 	opp_error("TraCI server \"%s\" reports API version %d, which is unsupported. We recommend using SUMO 0.21.0.", serverVersion.c_str(), apiVersion);
		// }

	}

	{
		// query road network boundaries
		TraCIBuffer buf = connection->query(CMD_GET_SIM_VARIABLE, TraCIBuffer() << static_cast<uint8_t>(VAR_NET_BOUNDING_BOX) << std::string("sim0"));
		uint8_t cmdLength_resp; buf >> cmdLength_resp;
		uint8_t commandId_resp; buf >> commandId_resp; ASSERT(commandId_resp == RESPONSE_GET_SIM_VARIABLE);
		uint8_t variableId_resp; buf >> variableId_resp; ASSERT(variableId_resp == VAR_NET_BOUNDING_BOX);
		std::string simId; buf >> simId;
		uint8_t typeId_resp; buf >> typeId_resp; ASSERT(typeId_resp == TYPE_BOUNDINGBOX);
		double x1; buf >> x1;
		double y1; buf >> y1;
		double x2; buf >> x2;
		double y2; buf >> y2;
		ASSERT(buf.eof());

		netbounds1 = TraCICoord(x1, y1);
		netbounds2 = TraCICoord(x2, y2);
		MYDEBUG << "TraCI reports network boundaries (" << x1 << ", " << y1 << ")-(" << x2 << ", " << y2 << ")" << endl;
		//connection->setNetbounds(netbounds1, netbounds2, par("margin"));
		//if ((connection->traci2omnet(netbounds2).x > world->getPgs()->x) || (connection->traci2omnet(netbounds1).y > world->getPgs()->y)) MYDEBUG << "WARNING: Playground size (" << world->getPgs()->x << ", " << world->getPgs()->y << ") might be too small for vehicle at network bounds (" << connection->traci2omnet(netbounds2).x << ", " << connection->traci2omnet(netbounds1).y << ")" << endl;
	}

	{
		// subscribe to list of departed and arrived vehicles, as well as simulation time
		uint32_t beginTime = 0;
		uint32_t endTime = 0x7FFFFFFF;
		std::string objectId = "";
		uint8_t variableNumber = 7;
		uint8_t variable1 = VAR_DEPARTED_VEHICLES_IDS;
		uint8_t variable2 = VAR_ARRIVED_VEHICLES_IDS;
		uint8_t variable3 = VAR_TIME_STEP;
		uint8_t variable4 = VAR_TELEPORT_STARTING_VEHICLES_IDS;
		uint8_t variable5 = VAR_TELEPORT_ENDING_VEHICLES_IDS;
		uint8_t variable6 = VAR_PARKING_STARTING_VEHICLES_IDS;
		uint8_t variable7 = VAR_PARKING_ENDING_VEHICLES_IDS;
		TraCIBuffer buf = connection->query(CMD_SUBSCRIBE_SIM_VARIABLE, TraCIBuffer() << beginTime << endTime << objectId << variableNumber << variable1 << variable2 << variable3 << variable4 << variable5 << variable6 << variable7);
		processSubcriptionResult(buf);
		ASSERT(buf.eof());
	}

	{
		// subscribe to list of vehicle ids
		uint32_t beginTime = 0;
		uint32_t endTime = 0x7FFFFFFF;
		std::string objectId = "";
		uint8_t variableNumber = 1;
		uint8_t variable1 = ID_LIST;
		TraCIBuffer buf = connection->query(CMD_SUBSCRIBE_VEHICLE_VARIABLE, TraCIBuffer() << beginTime << endTime << objectId << variableNumber << variable1);
		processSubcriptionResult(buf);
		ASSERT(buf.eof());
	}

	if (vehicleTypeIds.size()==0) {
		std::list<std::string> vehTypes = getCommandInterface()->getVehicleTypeIds();
		for (std::list<std::string>::const_iterator i = vehTypes.begin(); i != vehTypes.end(); ++i) {
			std::cout<<"vehicleTypeId : "<<*i<<std::endl;
			if (i->compare("DEFAULT_VEHTYPE")!=0 && i->compare("DEFAULT_PEDTYPE")!=0) {
				vehicleTypeIds.push_back(*i);
			}
		}
	}
	if (routeIds.size()==0) {
		std::list<std::string> routes = getCommandInterface()->getRouteIds();
		for (std::list<std::string>::const_iterator i = routes.begin(); i != routes.end(); ++i) {
			std::string routeId = *i;

			MYDEBUG << "Adding " << routeId << " to list of possible routes" << std::endl;
			routeIds.push_back(routeId);
		}
	}
}


std::map<int, vechile> TraCIScenarioManager::executeOneTimestep() {
	vechileResults.clear();
	std::cout << "Triggering TraCI server simulation advance to t=" << currentat <<std::endl;

	uint32_t targetTime = round(currentat * 1000);
	//std::cout<<" targetTime "<<targetTime <<" connectAt "<<connectAt<<" currentat "<<currentat<<std::endl;
	insertVehicles();
	if ( targetTime >= round(connectAt * 1000)) {
		TraCIBuffer buf = connection->query(CMD_SIMSTEP2, TraCIBuffer() << targetTime);

		uint32_t count; buf >> count;
		std::cout << "Getting " << count << " subscription results" << std::endl;
		for (uint32_t i = 0; i < count; ++i) {
			processSubcriptionResult(buf);
		}
	}
	currentat += updateInterval;
	return vechileResults;
}

void TraCIScenarioManager::checkTimeMoveSumoFoward(double targetTime) {
	uint32_t targetTime_int = floor(targetTime * 1000);
	//std::cout<<targetTime<<"currentat "<<currentat<<" targetTime_int "<<targetTime_int<<std::endl;
	if(targetTime_int==0){
		return;
	}
	if(targetTime_int!=round(currentat * 1000)) {
		ASSERT(targetTime_int - round(currentat * 1000)==round(updateInterval*1000));
		executeOneTimestep();
	}
}

Point TraCIScenarioManager::GetPosition(double targetTime, int id) {
	checkTimeMoveSumoFoward(targetTime);
	auto i =vechileResults.find(id);
	if(i!=vechileResults.end()) {
		return Point(i->second.x,i->second.y);
	} else if(ra2020IdToSumoIdMap_.find(id)!=ra2020IdToSumoIdMap_.end()){
		TraCICommandInterface::Vehicle v(getCommandInterface(),ra2020IdToSumoIdMap_[id]);
		return v.getVehiclePosition();
	} else {
		return Point(-1001,-1001);
	}

}

double TraCIScenarioManager::GetSpeed(double targetTime, int id) {
	checkTimeMoveSumoFoward(targetTime);
	auto i =vechileResults.find(id);
	if(i!=vechileResults.end()) {
		return i->second.velocity;
	} else if(ra2020IdToSumoIdMap_.find(id)!=ra2020IdToSumoIdMap_.end()){

		TraCICommandInterface::Vehicle v(getCommandInterface(),ra2020IdToSumoIdMap_[id]);
		return v.getSpeed();
	} else {
		return -1001;
	}

}

bool TraCIScenarioManager::isInRegionOfInterest(const TraCICoord& position, std::string road_id, double speed, double angle) {
	if ((roiRoads.size() == 0) && (roiRects.size() == 0)) return true;
	if (roiRoads.size() > 0) {
		for (std::list<std::string>::const_iterator i = roiRoads.begin(); i != roiRoads.end(); ++i) {
			if (road_id == *i) return true;
		}
	}
	if (roiRects.size() > 0) {
		for (std::list<std::pair<TraCICoord, TraCICoord> >::const_iterator i = roiRects.begin(); i != roiRects.end(); ++i) {
			if ((position.x >= i->first.x) && (position.y >= i->first.y) && (position.x <= i->second.x) && (position.y <= i->second.y)) return true;
		}
	}
	return false;
}

void TraCIScenarioManager::insertNewVehicle(int veh_id) {
	std::string type;
	if (vehicleTypeIds.size()) {
		int vehTypeId = randomStream.GetInt() % vehicleTypeIds.size();
		type = vehicleTypeIds[vehTypeId];
	}
	else {
		type = "DEFAULT_VEHTYPE";
	}
	int routeId =  randomStream.GetInt() % routeIds.size();
	vehicleInsertQueue[routeId].push(std::make_pair(veh_id,type));
}

void TraCIScenarioManager::insertVehicles() {

	for (auto i = vehicleInsertQueue.begin(); i != vehicleInsertQueue.end(); ) {
		std::string route = routeIds[i->first];
		std::cout << "process " << route << std::endl;
		auto vehicles = i->second;
		while (!i->second.empty()) {
			bool suc = false;
			std::string type = i->second.front().second;
			std::stringstream veh;
			veh << i->second.front().first;
			std::cout << "trying to add " << veh.str() << " with " << route << " vehicle type " << type << std::endl;

			suc = getCommandInterface()->addVehicle(veh.str(), type, route);
			if (!suc) {
				std::cout << "wrongly inserted " << veh.str() << std::endl;	
				i->second.pop();
			}
			else {
				std::cout << "successfly inserted " << veh.str() << std::endl;
				i->second.pop();
				vehicleNameCounter++;
			}
		}
		auto tmp = i;
		++tmp;
		vehicleInsertQueue.erase(i);
		i = tmp;

	}
}


void TraCIScenarioManager::subscribeToVehicleVariables(std::string vehicleId) {
	// subscribe to some attributes of the vehicle
	uint32_t beginTime = 0;
	uint32_t endTime = 0x7FFFFFFF;
	std::string objectId = vehicleId;
	uint8_t variableNumber = 5;
	uint8_t variable1 = VAR_POSITION;
	uint8_t variable2 = VAR_ROAD_ID;
	uint8_t variable3 = VAR_SPEED;
	uint8_t variable4 = VAR_ANGLE;
	uint8_t variable5 = VAR_SIGNALS;

	TraCIBuffer buf = connection->query(CMD_SUBSCRIBE_VEHICLE_VARIABLE, TraCIBuffer() << beginTime << endTime << objectId << variableNumber << variable1 << variable2 << variable3 << variable4 << variable5);
	processSubcriptionResult(buf);
	ASSERT(buf.eof());
}

void TraCIScenarioManager::unsubscribeFromVehicleVariables(std::string vehicleId) {
	// subscribe to some attributes of the vehicle
	uint32_t beginTime = 0;
	uint32_t endTime = 0x7FFFFFFF;
	std::string objectId = vehicleId;
	uint8_t variableNumber = 0;

	TraCIBuffer buf = connection->query(CMD_SUBSCRIBE_VEHICLE_VARIABLE, TraCIBuffer() << beginTime << endTime << objectId << variableNumber);
	ASSERT(buf.eof());
}


void TraCIScenarioManager::processSimSubscription(std::string objectId, TraCIBuffer& buf) {
	uint8_t variableNumber_resp; buf >> variableNumber_resp;
	for (uint8_t j = 0; j < variableNumber_resp; ++j) {
		uint8_t variable1_resp; buf >> variable1_resp;
		uint8_t isokay; buf >> isokay;
		if (isokay != RTYPE_OK) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_STRING);
			std::string description; buf >> description;
			if (isokay == RTYPE_NOTIMPLEMENTED) opp_error("TraCI server reported subscribing to variable 0x%2x not implemented (\"%s\"). Might need newer version.", variable1_resp, description.c_str());
			opp_error("TraCI server reported opp_error subscribing to variable 0x%2x (\"%s\").", variable1_resp, description.c_str());
		}

		if (variable1_resp == VAR_DEPARTED_VEHICLES_IDS) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_STRINGLIST);
			uint32_t count; buf >> count;
			MYDEBUG << "TraCI reports " << count << " departed vehicles." << endl;
			for (uint32_t i = 0; i < count; ++i) {
				std::string idstring; buf >> idstring;
				// adding modules is handled on the fly when entering/leaving the ROI
			}

			activeVehicleCount += count;
			drivingVehicleCount += count;

		} else if (variable1_resp == VAR_ARRIVED_VEHICLES_IDS) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_STRINGLIST);
			uint32_t count; buf >> count;
			MYDEBUG << "TraCI reports " << count << " arrived vehicles." << endl;
			for (uint32_t i = 0; i < count; ++i) {
				std::string idstring; buf >> idstring;

				if (subscribedVehicles.find(idstring) != subscribedVehicles.end()) {
					subscribedVehicles.erase(idstring);
					unsubscribeFromVehicleVariables(idstring);
				}


				if(unEquippedHosts.find(idstring) != unEquippedHosts.end()) {
					unEquippedHosts.erase(idstring);
				}

			}

			if ((count > 0) && (count >= activeVehicleCount) && autoShutdown) autoShutdownTriggered = true;
			activeVehicleCount -= count;
			drivingVehicleCount -= count;

		} else if (variable1_resp == VAR_TELEPORT_STARTING_VEHICLES_IDS) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_STRINGLIST);
			uint32_t count; buf >> count;
			MYDEBUG << "TraCI reports " << count << " vehicles starting to teleport." << endl;
			for (uint32_t i = 0; i < count; ++i) {
				std::string idstring; buf >> idstring;

				if(unEquippedHosts.find(idstring) != unEquippedHosts.end()) {
					unEquippedHosts.erase(idstring);
				}

			}

			activeVehicleCount -= count;
			drivingVehicleCount -= count;

		} else if (variable1_resp == VAR_TELEPORT_ENDING_VEHICLES_IDS) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_STRINGLIST);
			uint32_t count; buf >> count;
			MYDEBUG << "TraCI reports " << count << " vehicles ending teleport." << endl;
			for (uint32_t i = 0; i < count; ++i) {
				std::string idstring; buf >> idstring;
				// adding modules is handled on the fly when entering/leaving the ROI
			}

			activeVehicleCount += count;
			drivingVehicleCount += count;

		} else if (variable1_resp == VAR_PARKING_STARTING_VEHICLES_IDS) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_STRINGLIST);
			uint32_t count; buf >> count;
			MYDEBUG << "TraCI reports " << count << " vehicles starting to park." << endl;
			for (uint32_t i = 0; i < count; ++i) {
				std::string idstring; buf >> idstring;
			}

			parkingVehicleCount += count;
			drivingVehicleCount -= count;

		} else if (variable1_resp == VAR_PARKING_ENDING_VEHICLES_IDS) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_STRINGLIST);
			uint32_t count; buf >> count;
			MYDEBUG << "TraCI reports " << count << " vehicles ending to park." << endl;
			for (uint32_t i = 0; i < count; ++i) {
				std::string idstring; buf >> idstring;
			}
			parkingVehicleCount -= count;
			drivingVehicleCount += count;

		} else if (variable1_resp == VAR_TIME_STEP) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_INTEGER);
			uint32_t serverTimestep; buf >> serverTimestep;
			MYDEBUG << "TraCI reports current time step as " << serverTimestep << "ms." << endl;

		} else {
			opp_error("Received unhandled sim subscription result");
		}
	}
}



void TraCIScenarioManager::processVehicleSubscription(std::string objectId, TraCIBuffer& buf) {
	bool isSubscribed = (subscribedVehicles.find(objectId) != subscribedVehicles.end());
	double px;
	double py;
	std::string edge;
	double speed;
	double angle_traci;
	int signals;
	int numRead = 0;

	uint8_t variableNumber_resp; buf >> variableNumber_resp;
	for (uint8_t j = 0; j < variableNumber_resp; ++j) {
		uint8_t variable1_resp; buf >> variable1_resp;
		uint8_t isokay; buf >> isokay;
		if (isokay != RTYPE_OK) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_STRING);
			std::string opp_errormsg; buf >> opp_errormsg;
			if (isSubscribed) {
				if (isokay == RTYPE_NOTIMPLEMENTED) opp_error("TraCI server reported subscribing to vehicle variable 0x%2x not implemented (\"%s\"). Might need newer version.", variable1_resp, opp_errormsg.c_str());
				opp_error("TraCI server reported opp_error subscribing to vehicle variable 0x%2x (\"%s\").", variable1_resp, opp_errormsg.c_str());
			}
		} else if (variable1_resp == ID_LIST) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_STRINGLIST);
			uint32_t count; buf >> count;
			MYDEBUG << "TraCI reports " << count << " active vehicles." << endl;
			MYDEBUG << "TraCI reports " << drivingVehicleCount << " driving vehicles." << endl;
			ASSERT(count == drivingVehicleCount);
			std::set<std::string> drivingVehicles;
			for (uint32_t i = 0; i < count; ++i) {
				std::string idstring; buf >> idstring;
				MYDEBUG << "TraCI reports " << count << " active vehicles: " << idstring<< endl;
				drivingVehicles.insert(idstring);
			}

			// check for vehicles that need subscribing to
			std::set<std::string> needSubscribe;
			std::set_difference(drivingVehicles.begin(), drivingVehicles.end(), subscribedVehicles.begin(), subscribedVehicles.end(), std::inserter(needSubscribe, needSubscribe.begin()));
			for (std::set<std::string>::const_iterator i = needSubscribe.begin(); i != needSubscribe.end(); ++i) {
				subscribedVehicles.insert(*i);
				subscribeToVehicleVariables(*i);
			}

			// check for vehicles that need unsubscribing from
			std::set<std::string> needUnsubscribe;
			std::set_difference(subscribedVehicles.begin(), subscribedVehicles.end(), drivingVehicles.begin(), drivingVehicles.end(), std::inserter(needUnsubscribe, needUnsubscribe.begin()));
			for (std::set<std::string>::const_iterator i = needUnsubscribe.begin(); i != needUnsubscribe.end(); ++i) {
				subscribedVehicles.erase(*i);
				unsubscribeFromVehicleVariables(*i);
			}

		} else if (variable1_resp == VAR_POSITION) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == POSITION_2D);
			buf >> px;
			buf >> py;
			numRead++;
		} else if (variable1_resp == VAR_ROAD_ID) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_STRING);
			buf >> edge;
			numRead++;
		} else if (variable1_resp == VAR_SPEED) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_DOUBLE);
			buf >> speed;
			numRead++;
		} else if (variable1_resp == VAR_ANGLE) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_DOUBLE);
			buf >> angle_traci;
			numRead++;
		} else if (variable1_resp == VAR_SIGNALS) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_INTEGER);
			buf >> signals;
			numRead++;
		} else {
			opp_error("Received unhandled vehicle subscription result");
		}
	}

	// bail out if we didn't want to receive these subscription results
	if (!isSubscribed) return;

	// make sure we got updates for all attributes
	if (numRead != 5) return;
	std::cout<<"processVehicleSubscription "<<objectId	<<" px "<<px<<" py "<<py<<" speed "<<speed<<" angle "<<angle_traci<<std::endl;
	Vechile v;
	v.x=px;
	v.y=py;
	v.z=0.0;
	v.velocity=speed;
	v.angle=angle_traci;
	if(sumoIdToRa2020IdMap_.find(objectId)==sumoIdToRa2020IdMap_.end()) {
		sumoIdToRa2020IdMap_[objectId] = nowRa2020Id_;
		ra2020IdToSumoIdMap_[nowRa2020Id_] = objectId;
		nowRa2020Id_++;		
	}
	std::cout<<objectId<<" <-> "<<sumoIdToRa2020IdMap_[objectId]<<std::endl;
	vechileResults[sumoIdToRa2020IdMap_[objectId]]=v;
}

void TraCIScenarioManager::processSubcriptionResult(TraCIBuffer& buf) {
	uint8_t cmdLength_resp; buf >> cmdLength_resp;
	uint32_t cmdLengthExt_resp; buf >> cmdLengthExt_resp;
	uint8_t commandId_resp; buf >> commandId_resp;
	std::string objectId_resp; buf >> objectId_resp;

	if (commandId_resp == RESPONSE_SUBSCRIBE_VEHICLE_VARIABLE) processVehicleSubscription(objectId_resp, buf);
	else if (commandId_resp == RESPONSE_SUBSCRIBE_SIM_VARIABLE) processSimSubscription(objectId_resp, buf);
	else {
		opp_error("Received unhandled subscription result");
	}
}

double TraCIScenarioManager::getVehicleAngle(int id) {					// ADD cam traffic
	ASSERT(ra2020IdToSumoIdMap_.find(id)!=ra2020IdToSumoIdMap_.end());
	TraCICommandInterface::Vehicle v(getCommandInterface(),ra2020IdToSumoIdMap_[id]);
	return v.getAngle();
}

double TraCIScenarioManager::getVehicleAcceleration(int id) {
	ASSERT(ra2020IdToSumoIdMap_.find(id)!=ra2020IdToSumoIdMap_.end());
	TraCICommandInterface::Vehicle v(getCommandInterface(),ra2020IdToSumoIdMap_[id]);
	return v.getAcceleration();
}

double TraCIScenarioManager::getStationLength(int id) {
	ASSERT(ra2020IdToSumoIdMap_.find(id)!=ra2020IdToSumoIdMap_.end());
	TraCICommandInterface::Vehicle v(getCommandInterface(),ra2020IdToSumoIdMap_[id]);
	return v.getLength();
}

double TraCIScenarioManager::getStationWidth(int id) {
	ASSERT(ra2020IdToSumoIdMap_.find(id)!=ra2020IdToSumoIdMap_.end());
	TraCICommandInterface::Vehicle v(getCommandInterface(),ra2020IdToSumoIdMap_[id]);
	return v.getWidth();
}

int TraCIScenarioManager::getLights(int id) {
	ASSERT(ra2020IdToSumoIdMap_.find(id)!=ra2020IdToSumoIdMap_.end());
	TraCICommandInterface::Vehicle v(getCommandInterface(),ra2020IdToSumoIdMap_[id]);
	return v.getLights();
}

std::string TraCIScenarioManager::getLaneId(int id) {
	ASSERT(ra2020IdToSumoIdMap_.find(id)!=ra2020IdToSumoIdMap_.end());
	TraCICommandInterface::Vehicle v(getCommandInterface(),ra2020IdToSumoIdMap_[id]);
	return v.getLaneId();
}

std::string TraCIScenarioManager::getEdgeId(int id) {
	ASSERT(ra2020IdToSumoIdMap_.find(id)!=ra2020IdToSumoIdMap_.end());
	TraCICommandInterface::Vehicle v(getCommandInterface(),ra2020IdToSumoIdMap_[id]);
	return v.getRoadId();
}

// std::string TraCIScenarioManager::getJunctionId(int id) {
// 	ASSERT(ra2020IdToSumoIdMap_.find(id)!=ra2020IdToSumoIdMap_.end());
// 	TraCICommandInterface::Vehicle v(getCommandInterface(),ra2020IdToSumoIdMap_[id]);
// 	return v.getJunctionId();
// }