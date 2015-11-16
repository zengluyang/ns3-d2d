/** The main class for the mobility model.
 * \file       basic-models/mobility/mobility-model.h
 * \author     Jussi Turkka
 * \version    5G Simulator 0.1
 * \date       2014.12.04
 * \copyright  Copyright (C), 1998-2014, Huawei Tech. Co., Ltd.
 */

#ifndef _MODELS_MOBILITY_MOBILITY_MODEL_H_
#define _MODELS_MOBILITY_MOBILITY_MODEL_H_

#include "simcore/object.h"

#include "math/types.h"
#include "math/geometry/point.h"
#include "math/random/unifrnd.h"
#include "math/geometry/unifrnd-rectangle.h"
#include "basic-models/mobility/mobility-types.h"
#include "basic-models/mobility/interfaces/mobility-model-interface.h"

class MobilityModel : public Object<MobilityModel>, public IMobilityModel
{
public:

    MobilityModel(int network_idx, const std::vector<Point>& bounding_box, double speed_in_ms);
    virtual ~MobilityModel();

    void Start();
    void Stop();
    virtual void GetPosition(Point& position);
    //virtual MobilityType GetType() { return MobilityType::BASIC;}
    virtual double GetVelocity();

    virtual double       GetAngle(){return angle_;}            // ADD cam traffic
    virtual double       GetAcceleration(){return acceleration_;}
    virtual double       GetStaLength(){return sta_length_;}
    virtual double       GetStaWidth(){return sta_width_;}
    virtual int          GetLights(){return lights_;}
    virtual std::string  GetLaneId(){return lane_id_;}
    virtual std::string  GetEdgeId(){return edge_id_;}
    virtual std::string  GetJunctionId(){return junction_id_;}

    virtual Ptr<IMobilityModel> Clone() const;
    virtual int GetId () const { return id_; }
    virtual void SetId(int id) { id_ = id; }
private:
    bool CheckTriggers(Point& position);
    void CalculatePosition(Point& position);
    virtual void UpdateMovement(Point& position);

    Point velocity_;                  // direction where object is moving at speed of m/s
    double time_now_;                 // current time
    double last_move_time_;           // time since last location update
    double next_update_time_;         // time to new direction update
    double speed_ms_;                 // velocity of the object m/s
    std::vector<Point> bounding_box_; // area from where random location is obtained
    bool running_;
    UnifRndRectangle generate_point_;
    int id_;

    double                  angle_;
    double                  acceleration_;
    double                  sta_length_;
    double                  sta_width_;
    int                     lights_;
    std::string             lane_id_;
    std::string             edge_id_;
    std::string             junction_id_;
    static RandomStream r_waypoint_rs_;
};

#endif /*_MODELS_MOBILITY_MOBILITY_MODEL_H_*/

