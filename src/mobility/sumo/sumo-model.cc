#include "basic-models/mobility/sumo-model.h"
#include "basic-models/mobility/mobility-types.h"
#include "basic-models/mobility/sumo-tcl-parser.h"    
#include "basic-models/mobility/sumo-xml-parser.h"
#include "basic-models/mobility/traci/traci-scenario-manager.h"
#include "simcore/simulator.h"


#include <limits>       // std::numeric_limits

SumoModelType SumoModel::sumo_model_type;
SUMOParserType SumoModel::sumo_parser_type;
bool SumoModel::vehicle_generated_from_xml;
SumoModel::SumoModel() {

}

SumoModel::~SumoModel() {

}

void SumoModel::Init(const Parameters& par) {
  sumo_model_type = par.sumo.sumo_model_type;
  int BbuNumber = par.deployment.infra_node_.number_of_rrus;
  sumo_parser_type = par.sumo.sumoparsertype;
  vehicle_generated_from_xml= par.sumo.vehicle_generated_from_xml;
  if(sumo_model_type == SUMO_FILE)
  {   
    if(sumo_parser_type==XML)
    {
      SumoXmlParser::Init();

      SumoXmlPsr().setInitialVehicleId(BbuNumber);
      SumoXmlPsr().parse(par.sumo.filename);
      if(SumoXmlPsr().getVehiclesNumber()!=par.deployment.number_of_ues)
      {
        std::cout<<"Warning: the number of UE is not equal to the number in the sumo xml file."<<std::endl;
      }
      if(SumoXmlPsr().getTimestep()!=SimTime(par.sumo.updateInterval))
      {
        std::cout<<"Warning: the Timestep is not equal to Timestep to in the sumo xml file."<<std::endl;
      }
      if(SumoXmlPsr().getEndTime()!=SimTime(par.end_time))
      {
        std::cout<<"Warning: the EndTime is not equal to EndTime in the sumo xml file."<<std::endl;
      }
      SumoXmlPsr().ShowMap();
    }
    else if(sumo_parser_type==TCL)
    {
      SumoTclParser::Init();
      SumoTclPsr().setInitialVehicleId(BbuNumber);
      SumoTclPsr().parse(par.sumo.filename);
      if(SumoTclPsr().getVehiclesNumber()!=par.deployment.number_of_ues)
      {
        std::cout<<"Warning: the number of UE is not equal to the number in the sumo tcl file."<<std::endl;
      }
      if(SumoTclPsr().getTimestep()!=SimTime(par.sumo.updateInterval))
      {
        std::cout<<"Warning: the Timestep is not equal to Timestep to in the sumo tcl file."<<std::endl;
      }
      if(SumoTclPsr().getEndTime()!=SimTime(par.end_time))
      {
        std::cout<<"Warning: the EndTime is not equal to EndTime in the sumo tcl file."<<std::endl;
      }
    }
  }
  else if(sumo_model_type == SUMO_CLIENT)
  {
    TraCIScenarioManager::Init(par.sumo);
    TraCIScenarioMgr().setInitialVehicleId(BbuNumber);
    TraCIScenarioMgr().connectAndStartTrigger();
    if(!vehicle_generated_from_xml)
    {
      for(int i = BbuNumber;i<par.deployment.number_of_ues + BbuNumber;i++)
      {
        std::cout<<i<<std::endl;
        InsertVehicle(i);
      }
    }
    TraCIScenarioMgr().executeOneTimestep();
  }

}

double SumoModel::GetVelocity() {
  double speed;
  if(sumo_model_type==SumoModelType::SUMO_FILE)
  {
    if(sumo_parser_type==SUMOParserType::XML)
    {
      speed = SumoXmlPsr().getSpeed(Simulator::Now(),id_); 
    }
    else
    {
      speed = SumoTclPsr().getSpeed(Simulator::Now(),id_); 
    }

  }
  else
  {
    speed = TraCIScenarioMgr().GetSpeed(Simulator::Now(),id_); 
    if(speed==-1001){
      std::cout<<"speed "<<id_<<" "<<speed<<std::endl;
      return speed;
    }
  }
  return speed;
}

void SumoModel::GetPosition(Point& position) {
  if(sumo_model_type==SumoModelType::SUMO_FILE)
  {
    if(sumo_parser_type==SUMOParserType::XML)
    {
      position = SumoXmlPsr().getPosition(Simulator::Now(),id_); 
    }
    else
    {
      position = SumoTclPsr().getPosition(Simulator::Now(),id_); 
    }
  }
  else
  {
    position = TraCIScenarioMgr().GetPosition(Simulator::Now(),id_);
    if(position==Point(-1001,-1001))
    {
      //position = Point(-std::numeric_limits<double>::infinity(),
      //-std::numeric_limits<double>::infinity());
      position = Point(ra2020_bouding_box[2].x+1.0,ra2020_bouding_box[2].y+1.0);
      std::cout<<"position "<<id_<<" "<<position<<std::endl;
      return;
    }
  }
  position = PostionSumoToRa2020(position);
  std::cout<<"position "<<id_<<" "<<position<<std::endl;
}

double SumoModel::GetAngle()                  // ADD cam traffic
{
  double angle = 0.0;
  if(sumo_model_type==SumoModelType::SUMO_FILE)
  {
    if(sumo_parser_type==SUMOParserType::XML)
    {
      return angle; 
    }
    else
    {
      return angle; 
    }

  }
  else
  {
    angle = TraCIScenarioMgr().getVehicleAngle(id_); 
    return angle;
  }
}
double SumoModel::GetAcceleration()
{
  double acceleration = 0.0;
  if(sumo_model_type==SumoModelType::SUMO_FILE)
  {
    if(sumo_parser_type==SUMOParserType::XML)
    {
      return acceleration; 
    }
    else
    {
      return acceleration; 
    }

  }
  else
  {
    acceleration = TraCIScenarioMgr().getVehicleAcceleration(id_); 
    return acceleration;
  }
}
double SumoModel::GetStaLength()
{
  double sta_length = 0.0;
  if(sumo_model_type==SumoModelType::SUMO_FILE)
  {
    if(sumo_parser_type==SUMOParserType::XML)
    {
      return sta_length; 
    }
    else
    {
      return sta_length; 
    }

  }
  else
  {
    sta_length = TraCIScenarioMgr().getStationLength(id_); 
    return sta_length;
  }
}
double SumoModel::GetStaWidth()
{
  double sta_width = 0.0;
  if(sumo_model_type==SumoModelType::SUMO_FILE)
  {
    if(sumo_parser_type==SUMOParserType::XML)
    {
      return sta_width; 
    }
    else
    {
      return sta_width; 
    }

  }
  else
  {
    sta_width = TraCIScenarioMgr().getStationWidth(id_); 
    return sta_width;
  }
}
int SumoModel::GetLights()
{
  int ligths = 0;
  if(sumo_model_type==SumoModelType::SUMO_FILE)
  {
    if(sumo_parser_type==SUMOParserType::XML)
    {
      return ligths; 
    }
    else
    {
      return ligths; 
    }

  }
  else
  {
    ligths = TraCIScenarioMgr().getLights(id_); 
    return ligths;
  }
}
std::string SumoModel::GetLaneId()
{
  std::string lane_id;
  if(sumo_model_type==SumoModelType::SUMO_FILE)
  {
    if(sumo_parser_type==SUMOParserType::XML)
    {
      return lane_id; 
    }
    else
    {
      return lane_id; 
    }

  }
  else
  {
    lane_id = TraCIScenarioMgr().getLaneId(id_); 
    return lane_id;
  }
}
std::string SumoModel::GetEdgeId()
{
  std::string edge_id;
  if(sumo_model_type==SumoModelType::SUMO_FILE)
  {
    if(sumo_parser_type==SUMOParserType::XML)
    {
      return edge_id; 
    }
    else
    {
      return edge_id; 
    }

  }
  else
  {
    edge_id = TraCIScenarioMgr().getEdgeId(id_); 
    return edge_id;
  }
}
std::string SumoModel::GetJunctionId()              // ADD cam traffic
{
  std::string junction_id;
  if(sumo_model_type==SumoModelType::SUMO_FILE)
  {
    if(sumo_parser_type==SUMOParserType::XML)
    {
      return junction_id; 
    }
    else
    {
      return junction_id; 
    }

  }
  else
  {
    // junction_id = TraCIScenarioMgr().getJunctionId(id_);
    junction_id = "0"; 
    return junction_id;
  }
}

void SumoModel::InsertVehicle(int id) {
  if(sumo_model_type==SumoModelType::SUMO_FILE)
  {
    return;
  }
  else
  {
    if(vehicle_generated_from_xml)
    {
      return;
    }
    else
    {
      TraCIScenarioMgr().insertNewVehicle(id);
    }
  }

}

UnifRndRectangle*  SumoModel::generate_point_;

Point SumoModel::GetInitialPostion(int id)
{
  Point position;
  if(sumo_model_type==SumoModelType::SUMO_FILE)
  {
    position = SumoXmlPsr().getPosition(0.0,id);
    std::cout<<"init position "<<position<<std::endl;
    return PostionSumoToRa2020(position);
  } 
  else 
  {
    position = TraCIScenarioMgr().GetPosition(0.0,id);
    if(position == Point(-1001,-1001))
    {
      position = Point(ra2020_bouding_box[2].x+1.0,ra2020_bouding_box[2].y+1.0);
    }
    else
    {
      position = PostionSumoToRa2020(position);
    }
    std::cout<<"init position "<<position<<std::endl;
    return position;
  }
}

//Create a new mobility model object of the same type.
//The state of the cloned object is reset. Initialize must be called to use the channel.
 Ptr<IMobilityModel> SumoModel::Clone() const
 {
  return ::Create< SumoModel >(*this);
}


double SumoModel::vertical_offset = 0.0;
double SumoModel::horizontal_offset = 0.0;
std::vector<Point> SumoModel::sumo_bouding_box;
std::vector<Point> SumoModel::ra2020_bouding_box;

//check if the bouding box of sumo and the bouding box of RA2020 can fit
bool SumoModel::mapSizeFit(std::vector<Point> v)
{
  //v is from RA2020
  ra2020_bouding_box = v;
  std::cout<<"RA2020 bouding box "<<ra2020_bouding_box[0]<<" "<<ra2020_bouding_box[2]<<std::endl;
  sumo_bouding_box = getBoundingBox();
  if ( ( ( v[2].x - v[0].x ) > ( sumo_bouding_box[1].x - sumo_bouding_box[0].x ) ) && ( ( v[2].y - v[0].y ) > ( sumo_bouding_box[1].y - sumo_bouding_box[0].y ) ) ){
    vertical_offset = ( sumo_bouding_box[0].y + sumo_bouding_box[1].y ) / 2.0;
    horizontal_offset = ( sumo_bouding_box[0].x + sumo_bouding_box[1].x ) / 2.0;
    std::cout<<"vertical_offset "<<vertical_offset<<" horizontal_offset "<<horizontal_offset<<std::endl;
    return true;
  }
  else {
    return false;
  }
}

std::vector<Point> SumoModel::getBoundingBox()
{
  std::vector<Point> v;
  switch (sumo_model_type)
  {
    case SUMO_FILE:
      if(sumo_parser_type == XML)
      {
        v.push_back(SumoXmlPsr().getDownLeftPoint());
        v.push_back(SumoXmlPsr().getTopRightPoint());
        return v;
      }
      else if(sumo_parser_type == TCL)
      {
        v.push_back(SumoTclPsr().getDownLeftPoint());
        v.push_back(SumoTclPsr().getTopRightPoint());
        return v;
      }
      else
      {
        ASSERT("error!");
      }
      break;
    case SUMO_CLIENT:
      v = TraCIScenarioMgr().getBoundingBox();
      return v;
      break;
    default:
      ASSERT("error!");
  }
}

Point SumoModel::PostionSumoToRa2020(Point sumo)
{
  return Point(sumo.x-horizontal_offset,sumo.y-vertical_offset);
}