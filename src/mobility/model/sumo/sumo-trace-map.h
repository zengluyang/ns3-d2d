
#ifndef SUMO_TRACE_MAP_H
#define SUMO_TRACE_MAP_H


#include <map>
#include <vector>

#include "simcore/simtime.h"

using namespace std;

typedef struct vechile
{
	int id;
	SimTime time;
	double velocity;
	double x;
	double y;
	double z;
	double angle;
}  Vechile;

typedef std::map<SimTime,vector<Vechile> >  SUMOTraceMap;

#endif