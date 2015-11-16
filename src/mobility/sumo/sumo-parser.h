#ifndef _SUMO_PARSER_H_
#define _SUMO_PARSER_H_



#include <map>
#include <vector>
#include <iostream>
#include <cstring>

#include "basic-models/mobility/sumo-trace-map.h"
#include "math/geometry/point.h"
#include "simcore/simtime.h"

class SumoParser {

public:
	SUMOTraceMap trace;
	
	void ShowMap();
	Point getPosition(double time,int vechile_id);
	Point getPosition(SimTime time,int vechile_id);
	double getSpeed(double time,int vechile_id);
	double getSpeed(SimTime time,int vechile_id);
	map<int,Point> getAllPositionsAtTime(double time);
	map<int,Point> getAllPositionsAtTime(SimTime time);
	map<int,double> getAllSpeedAtTime(double time);
	map<int,double> getAllSpeedAtTime(SimTime time);
	SimTime getTimestep();
	int getVehiclesNumber();
	void setInitialVehicleId(int id) { nowRa2020Id_ = id; initialId_ =id;}
    SimTime getEndTime() { 
        auto it = trace.cend();
        it--;
        return it->first;
    }
    double xMax_ ;
    double yMax_ ;
    double xMin_ ;
    double yMin_ ;

    void findMinMax();
	Point getTopRightPoint() { return Point(xMax_,yMax_); }// for the boudingBox
    Point getDownLeftPoint() { return Point(xMin_,yMin_); }// for the boudingBox
protected:
	int nowRa2020Id_ = 0;
	int initialId_ = 0;
private:
	SimTime FindLeastDiffTime(SimTime time);
};


#endif