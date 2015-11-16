#include "basic-models/mobility/sumo-xml-parser.h"
#include "basic-models/mobility/rapidxml/rapidxml.hpp"
#include "basic-models/mobility/rapidxml/rapidxml_utils.hpp"
#include "simcore/utils/print.h"
#include <algorithm>
#include <iostream>
#include <string>
#include <stdexcept>
SumoXmlParser* SumoXmlParser::inst_ = 0;


SumoXmlParser::SumoXmlParser() 
{

}

void SumoXmlParser::Init() {
  SumoXmlParser::inst_ = new SumoXmlParser();
}

void SumoXmlParser::DeleteInstance()
{
    delete inst_;
    inst_ = 0;
}

SUMOTraceMap SumoXmlParser::parse(std::string filename) {
	using namespace rapidxml;
	xml_document<> doc;    // character type defaults to char
	try {
		rapidxml::file<char> f(filename.c_str());
		
	} catch(const std::runtime_error& e){
		FATAL(e.what());
	}
	rapidxml::file<char> f(filename.c_str());
	doc.parse<0>(f.data());    // 0 means default parse flags 

	for(xml_node<> *timestep = doc.first_node("timestep");timestep;timestep=timestep->next_sibling()){
		//std::cout << timestep->name() << std::endl;
		xml_attribute<> *time = timestep->first_attribute("time");
		SimTime simTime  = SimTime(time->value());
		//std::cout<<time->name()<<" "<<time->value()<<" "<<simTime<<std::endl;
		for(xml_node<> *vehicle = timestep->first_node("vehicle");vehicle;vehicle=vehicle->next_sibling()) {
			//std::cout <<"\t"<< vehicle->name() << std::endl;
			int id=0;
			double x=0.0;
			double y=0.0;
			double z=0.0;
			double speed=0.0;

			for (xml_attribute<> *attr = vehicle->first_attribute();attr; attr = attr->next_attribute()){
			    //cout << "\t\t"<<attr->name() << " "<< attr->value() << "\n";
			    std::string name(attr->name());
			    std::string value(attr->value());
				if("id"==name) {
					if(sumoIdToRa2020IdMap_.find(value)==sumoIdToRa2020IdMap_.end()) {
						id = nowRa2020Id_;
						sumoIdToRa2020IdMap_[value] = id;
						ra2020IdToSumoIdMap_[id] = value;
						nowRa2020Id_++;
					} else {
						id = sumoIdToRa2020IdMap_[value];
					}
				} else if ("x"==name) {
					x = atof(value.c_str());
				} else if ("y"==name) {
					y = atof(value.c_str());
				} else if ("speed"==name) {
					speed = atof(value.c_str());
				}
			}
			//std::cout<<simTime<<" "<<id<<" "<<x<<" "<<y<<""<<z<<" "<<speed<<std::endl;
			Vechile v;
			v.id=id;
			v.time=simTime;	
			v.velocity=speed;
			v.x=x;
			v.y=y;
			v.z=z;
			trace[simTime].push_back(v);
		}
	}
	findMinMax();
	return trace;
}

//If ra2020Id does not match the key of any element in the container, the function throws an out_of_range exception
std::string SumoXmlParser::findSumoIdByRa2020Id(int ra2020Id) {
	return ra2020IdToSumoIdMap_.at(ra2020Id);
}

//If sumoId does not match the key of any element in the container, the function throws an out_of_range exception
int SumoXmlParser::findRa2020IdBySumoId(std::string sumoId) {
	return sumoIdToRa2020IdMap_.at(sumoId);
}

void SumoXmlParser::showSumoIdToRa2020IdMap() {
	for(auto it=sumoIdToRa2020IdMap_.cbegin();it!=sumoIdToRa2020IdMap_.cend();it++) {
		std::cout<<it->first<<"<--->"<<it->second<<std::endl;
	}
}