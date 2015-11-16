#ifndef _TRACI_CONNECTION_H_
#define _TRACI_CONNECTION_H_

#include <stdint.h>
#include "./traci-buffer.h"
#include "./traci-coord.h"
//#include "veins/base/utils/Coord.h"

void opp_error(const char *format, ...);

class TraCIConnection
{
	public:
		static TraCIConnection* connect(const char* host, int port);
		void setNetbounds(TraCICoord netbounds1, TraCICoord netbounds2, int margin);
		~TraCIConnection();

		/**
		 * sends a single command via TraCI, checks status response, returns additional responses
		 */
		TraCIBuffer query(uint8_t commandId, const TraCIBuffer& buf = TraCIBuffer());

		/**
		 * sends a single command via TraCI, expects no reply, returns true if successful
		 */
		TraCIBuffer queryOptional(uint8_t commandId, const TraCIBuffer& buf, bool& success, std::string* errorMsg = 0);

		/**
		 * sends a message via TraCI (after adding the header)
		 */
		void sendMessage(std::string buf);

		/**
		 * receives a message via TraCI (and strips the header)
		 */
		std::string receiveMessage();

		/**
		 * convert TraCI angle to OMNeT++ angle (in rad)
		 */
		double traci2omnetAngle(double angle) const;

		/**
		 * convert OMNeT++ angle (in rad) to TraCI angle
		 */
		double omnet2traciAngle(double angle) const;

	private:
		TraCIConnection(void*);

		void* socketPtr;
		TraCICoord netbounds1; /* network boundaries as reported by TraCI (x1, y1) */
		TraCICoord netbounds2; /* network boundaries as reported by TraCI (x2, y2) */
		int margin;

};

/**
 * returns byte-buffer containing a TraCI command with optional parameters
 */
std::string makeTraCICommand(uint8_t commandId, const TraCIBuffer& buf = TraCIBuffer());


#endif /* _TRACI_CONNECTION_H_ */
