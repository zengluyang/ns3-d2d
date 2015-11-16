#ifndef _SUMO_XML_PARSER_H_
#define _SUMO_XML_PARSER_H_

#include <map>
#include <string>

#include "basic-models/mobility/sumo-trace-map.h"
#include "basic-models/mobility/sumo-parser.h"
#include "math/geometry/point.h"

#include "basic-models/mobility/rapidxml/rapidxml.hpp"

class SumoXmlParser : public SumoParser{
public:
	SUMOTraceMap parse(std::string filename);
	std::string findSumoIdByRa2020Id(int ra2020Id);
	int findRa2020IdBySumoId(std::string sumoId);
	static void Init();
	static void DeleteInstance();
	static SumoXmlParser* Instance() { return inst_; }
	void showSumoIdToRa2020IdMap();
private:
    static SumoXmlParser* inst_;
    SumoXmlParser();
    SumoXmlParser( const SumoXmlParser& );             // disabled
    SumoXmlParser& operator=( const SumoXmlParser& );  // disabled
    std::map<std::string,int> sumoIdToRa2020IdMap_;
    std::map<int,std::string> ra2020IdToSumoIdMap_;
};


inline SumoXmlParser& SumoXmlPsr()
{
  return *SumoXmlParser::Instance();
}


#endif