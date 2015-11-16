#ifndef _TRACI_SCENARIO_MANAGER_H_
#define _TRACI_SCENARIO_MANAGER_H_


#include <map>
#include <set>
#include <list>
#include <queue>
#include <vector>
#include "./traci-buffer.h"
#include "./traci-color.h"
#include "./traci-connection.h"
#include "./traci-coord.h"
#include "./traci-command-interface.h"
#include "../sumo-trace-map.h"
#include "io/parameters.h"
#include "io/sumo-input-parameters.h"

#include "math/random/randomstream.h"
#include "math/geometry/point.h"

class TraCICommandInterface;
class TraCIScenarioManager{
	public:

		enum VehicleSignal {
			VEH_SIGNAL_UNDEF = -1,
			VEH_SIGNAL_NONE = 0,
			VEH_SIGNAL_BLINKER_RIGHT = 1,
			VEH_SIGNAL_BLINKER_LEFT = 2,
			VEH_SIGNAL_BLINKER_EMERGENCY = 4,
			VEH_SIGNAL_BRAKELIGHT = 8,
			VEH_SIGNAL_FRONTLIGHT = 16,
			VEH_SIGNAL_FOGLIGHT = 32,
			VEH_SIGNAL_HIGHBEAM = 64,
			VEH_SIGNAL_BACKDRIVE = 128,
			VEH_SIGNAL_WIPER = 256,
			VEH_SIGNAL_DOOR_OPEN_LEFT = 512,
			VEH_SIGNAL_DOOR_OPEN_RIGHT = 1024,
			VEH_SIGNAL_EMERGENCY_BLUE = 2048,
			VEH_SIGNAL_EMERGENCY_RED = 4096,
			VEH_SIGNAL_EMERGENCY_YELLOW = 8192
		};

		virtual ~TraCIScenarioManager();
		void initialize(const SUMOInputParameters& p);
		bool isConnected() const { return (connection); }
		TraCICommandInterface* getCommandInterface() const { return commandIfc; }

		bool getAutoShutdownTriggered() {
			return autoShutdownTriggered;
		}
		void connectAndStartTrigger();
		std::map<int, vechile> executeOneTimestep(); /**< read and execute all commands for the next timestep */

		/**
		 * adds a new vehicle to the queue which are tried to be inserted at the next SUMO time step;
		 */
		
		void insertNewVehicle(int veh_id);


		/**
		 * tries to add all vehicles in the vehicle queue to SUMO;
		 */
		void insertVehicles();
		Point GetPosition(double time, int id);
		double GetSpeed(double time, int id);
		static void Init(const SUMOInputParameters& p);
		static TraCIScenarioManager* Instance() { return inst_; }
		static void DeleteInstance();
		double GetCurrentTime() { return currentat;}
		std::vector<Point> getBoundingBox(){
			std::vector<Point> v;
			v.push_back(Point(netbounds1.x,netbounds1.y));
			v.push_back(Point(netbounds2.x,netbounds2.y));
			return v;
		}
		void setInitialVehicleId(int id) { nowRa2020Id_ = id; initialId_ = id;}
		void checkTimeMoveSumoFoward(double targetTime);
		double getVehicleAngle(int id);		// ADD cam traffic	TODO: mobility to be completed
		double getVehicleAcceleration(int id);
		double getStationLength(int id);
		double getStationWidth(int id);
		int    getLights(int id);
	    std::string   getLaneId(int id);
	    std::string   getEdgeId(int id);
	    // std::string   getJunctionId(int id);
	    void   setTraciInterface(unsigned int);
	protected:
	    static TraCIScenarioManager* inst_;
	    TraCIScenarioManager();
	    TraCIScenarioManager( const TraCIScenarioManager& );             // disabled
	    TraCIScenarioManager& operator=( const TraCIScenarioManager& );  // disabled
		double connectAt; /**< when to connect to TraCI server (must be the initial timestep of the server) */
		double firstStepAt; /**< when to start synchronizing with the TraCI server (-1: immediately after connecting) */
		double updateInterval; /**< time interval of hosts' position updates */
		double currentat;

		SUMOParserType sumoparsertype;	//added new;
		std::string filename;
		
		std::string host;
		int port;	

		uint32_t vehicleNameCounter;
		std::vector<std::string> vehicleTypeIds;
		std::map<int, std::queue<std::pair<int, std::string> > > vehicleInsertQueue;
		std::vector<std::string> routeIds;
		int vehicleRngIndex;
		int numVehicles;

		//int* mobRng;
		RandomStream randomStream;
		std::list<std::string> roiRoads; /**< which roads (e.g. "hwy1 hwy2") are considered to consitute the region of interest, if not empty */
		std::list<std::pair<TraCICoord, TraCICoord> > roiRects; /**< which rectangles (e.g. "0,0-10,10 20,20-30,30) are considered to consitute the region of interest, if not empty */

		TraCIConnection* connection;
		TraCICommandInterface* commandIfc;

		std::set<std::string> unEquippedHosts;
		std::set<std::string> subscribedVehicles; /**< all vehicles we have already subscribed to */
		uint32_t activeVehicleCount; /**< number of vehicles, be it parking or driving **/
		uint32_t parkingVehicleCount; /**< number of parking vehicles, derived from parking start/end events */
		uint32_t drivingVehicleCount; /**< number of driving, as reported by sumo */
		bool autoShutdownTriggered;
		bool autoShutdown;
		std::map<int, Vechile> vechileResults;
		std::string launchConfig; /**< launch configuration to send to sumo-launchd */
		int seed; /**< seed value to set in launch configuration, if missing (-1: current run number) */
		double penetrationRate;
		TraCICoord netbounds1;
		TraCICoord netbounds2;
		int nowRa2020Id_ = 0;
		int initialId_ = 0;
	    std::map<std::string,int> sumoIdToRa2020IdMap_;
	    std::map<int,std::string> ra2020IdToSumoIdMap_;
		virtual void init_traci();

		bool isInRegionOfInterest(const TraCICoord& position, std::string road_id, double speed, double angle);

		void subscribeToVehicleVariables(std::string vehicleId);
		void unsubscribeFromVehicleVariables(std::string vehicleId);
		void processSimSubscription(std::string objectId, TraCIBuffer& buf);
		void processVehicleSubscription(std::string objectId, TraCIBuffer& buf);
		void processSubcriptionResult(TraCIBuffer& buf);
};

inline TraCIScenarioManager& TraCIScenarioMgr()
{
  return *TraCIScenarioManager::Instance();
}


#endif