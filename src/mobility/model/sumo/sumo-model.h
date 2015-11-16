#ifndef _SUMO_MODEL_H_
#define _SUMO_MODEL_H_

#include <map>
#include <string>

#include "basic-models/mobility/sumo-trace-map.h"
#include "basic-models/mobility/sumo-parser.h"
#include "math/geometry/point.h"
#include "basic-models/mobility/interfaces/mobility-model-interface.h"
#include "io/parameters.h"
#include "basic-models/mobility/sumo-xml-parser.h"
#include "basic-models/mobility/mobility-types.h"
#include "math/random/randomstream.h"


class SumoModel : public Object<SumoModel>, public IMobilityModel {
public:
  SumoModel();
  virtual ~SumoModel();
  static void Init(const Parameters& par);
  static Point GetInitialPostion(int id);
  static void InsertVehicle(int id);
  static bool mapSizeFit(std::vector<Point> v);
  static std::vector<Point> getBoundingBox();
  static Point PostionSumoToRa2020(Point sumo);
  static void SetGeneratePoint(UnifRndRectangle* generate_point) {generate_point_ = generate_point;}

  virtual Ptr<IMobilityModel> Clone() const;

  virtual double GetVelocity();
  virtual void GetPosition(Point& position);
  virtual double       GetAngle();            // ADD cam traffic
  virtual double       GetAcceleration();
  virtual double       GetStaLength();
  virtual double       GetStaWidth();
  virtual int          GetLights();
  virtual std::string  GetLaneId();
  virtual std::string  GetEdgeId();
  virtual std::string  GetJunctionId();
  
  virtual MobilityType GetType() { return MobilityType::SUMO_MODEL;}
  virtual int GetId () const { return id_; }
  virtual void SetId(int id) { id_ = id; }
private:
  int id_;
  static SumoModelType sumo_model_type;
  static SUMOParserType sumo_parser_type;
  static bool vehicle_generated_from_xml;
  static double vertical_offset;
  static double horizontal_offset;
  static std::vector<Point> sumo_bouding_box;
  static std::vector<Point> ra2020_bouding_box;
  static UnifRndRectangle* generate_point_;
};

#endif