#include "basic-models/mobility/sumo-parser.h"
#include "simcore/dassert.h"
#include <iostream>

void 
SumoParser::ShowMap()
{
    std::cout<<"SumoTclParser::ShowMap()"<<std::endl;
    for(auto it=trace.cbegin();it!=trace.cend();it++)
    {
      for(auto iter = it->second.cbegin();iter!=it->second.cend();iter++)
      {
        std::cout<<it->first<<" "
                  <<(*iter).id<<" "
                  <<(*iter).time<<" "
                  <<(*iter).velocity<<" "
                  <<(*iter).x<<" "
                  <<(*iter).y<<" "
                  <<(*iter).z
                  <<std::endl;
      }
    }
}


SimTime SumoParser::FindLeastDiffTime(SimTime time){
  SimTime leastDiffTime;
  for(auto it=trace.cbegin();it!=trace.cend();it++) {
    SimTime t = it->first;
    if(t >= time) {
      break;
    }
    leastDiffTime = t;
  }
  return leastDiffTime;
}

Point SumoParser::getPosition(double time,int vechile_id){
  std::cout<<"Point SumoParser::getPositionn(double time,int vechile_id)"<<std::endl;
  return getPosition(SimTime(time),vechile_id);
}

Point SumoParser::getPosition(SimTime time,int vechile_id){
  std::cout<<"Point SumoParser::getPosition(SimTime time,int vechile_id) "<<vechile_id<<std::endl;
  SimTime leastDiffTime = this->FindLeastDiffTime(time);
  std::cout<<"leastDiffTime "<<leastDiffTime<<" "<<vechile_id<<std::endl;
  Vechile v=trace[leastDiffTime].at(vechile_id - initialId_);
  std::cout<<"x,y "<<v.x<<" "<<v.y<<std::endl;
  return Point(v.x,v.y,v.z);
}


double SumoParser::getSpeed(double time,int vechile_id) {
  return getSpeed(SimTime(time),vechile_id);
}

double SumoParser::getSpeed(SimTime time,int vechile_id) {
  SimTime leastDiffTime = this->FindLeastDiffTime(time);
  Vechile v=trace[leastDiffTime].at(vechile_id - initialId_);
  return v.velocity;
}

map<int,Point> SumoParser::getAllPositionsAtTime(double time){
  return getAllPositionsAtTime(SimTime(time));
}

map<int,Point> SumoParser::getAllPositionsAtTime(SimTime time){
  SimTime leastDiffTime = this->FindLeastDiffTime(time);
  auto vs = trace[leastDiffTime];
  map<int,Point> rlt;
  for(auto it=vs.cbegin();it!=vs.cend();it++) {
    rlt[it->id] = Point(it->x,it->y,it->z);
  }
  return rlt;
}

map<int,double> SumoParser::getAllSpeedAtTime(double time){
  return getAllSpeedAtTime(SimTime(time));
}

map<int,double> SumoParser::getAllSpeedAtTime(SimTime time){
  SimTime leastDiffTime = this->FindLeastDiffTime(time);
  auto vs = trace[leastDiffTime];
  map<int,double> rlt;
  for(auto it=vs.cbegin();it!=vs.cend();it++) {
    rlt[it->id] = it->velocity;
  }
  return rlt;
}

SimTime SumoParser::getTimestep(){
  ASSERT(trace.size()>=3);
  auto it = trace.cbegin();
  ++it;
  SimTime second = it->first;
  ++it;
  SimTime third = it->first;
  return third -second; //third time - secoud time
}

int SumoParser::getVehiclesNumber() {
  ASSERT(trace.size()>=1);
  auto it = trace.cbegin();
  return it->second.size();
}

void SumoParser::findMinMax() {
  for(auto i = trace.cbegin(); i != trace.cend(); i++) {
    auto v = i->second;
    xMax_ = v.cbegin()->x;
    yMax_ = v.cbegin()->y;
    xMin_ = v.cbegin()->x;
    yMin_ = v.cbegin()->y;
    for(auto vi = v.cbegin(); vi != v.cend(); vi++) {
      if( xMax_ > vi->x ) {
        xMax_ = vi->x;
      }
      if( yMax_ > vi->y ) {
        yMax_ = vi->y;
      }
      if( xMin_ < vi->x ) {
        xMin_ = vi->x;
      }
      if( xMin_ < vi->x ) {
        xMin_ = vi->x;
      }
    }
  }
}