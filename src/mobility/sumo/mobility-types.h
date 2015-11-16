#ifndef _MODELS_MOBILITY_TYPES_H_
#define _MODELS_MOBILITY_TYPES_H_
enum MobilityType { NOT_DEFINED = -1,
              	  	  NO_MOBILITY,
              	  	  BASIC,
              	  	  SUMO_MODEL};


enum SumoModelType 
{
	SUMO_FILE,
	SUMO_CLIENT
};

enum SUMOParserType
{
    TCL,
    XML
};

#endif