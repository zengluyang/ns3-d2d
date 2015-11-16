#ifndef _TRACI_COORD_H_
#define _TRACI_COORD_H_


/**
 * Coord equivalent for storing TraCI coordinates
 */
struct TraCICoord {
	TraCICoord() : x(0.0), y(0.0) {}
	TraCICoord(double x, double y) : x(x), y(y) {}
	double x;
	double y;
};


#endif /* _TRACI_COORD_H_ */