/** The main class for the mobility model.
 * \file       basic-models/mobility/mobility-model.h
 * \author     Jussi Turkka
 * \version    5G Simulator 0.1
 * \date       2014.12.04
 * \copyright  Copyright (C), 1998-2014, Huawei Tech. Co., Ltd.
 */
#include <iomanip>

#include "simcore/ptr.h"
#include "simcore/simulator.h"
#include "simcore/dassert.h"
#include "simcore/checked-enum.h"
#include "math/geometry/point.h"
#include "basic-models/mobility/mobility-model.h"
#include "math/random/rng-manager.h"

RandomStream MobilityModel::r_waypoint_rs_;

// template<>
// Enum<MobilityModel::Type>::values_t Enum<MobilityModel::Type>::values_
// {
//     { MobilityModel::NOT_DEFINED, "NOT_DEFINED" },
//     { MobilityModel::NO_MOBILITY, "NO_MOBILITY" },
//     { MobilityModel::BASIC,       "BASIC" }
// };

MobilityModel::MobilityModel(int network_idx, const std::vector<Point>& bounding_box, double speed_in_ms) :

    velocity_(),
    time_now_(),
    last_move_time_(0),
    next_update_time_(0),
    speed_ms_(speed_in_ms),
    bounding_box_(bounding_box),
    running_(false),
    angle_(0.0),       //ADD cam traffic
    acceleration_(0.0),
    sta_length_(0.0),
    sta_width_(0.0),
    lights_(0),
    lane_id_(),
    edge_id_(),
    junction_id_(),
    generate_point_(bounding_box_[0].x, bounding_box_[2].x,
                    bounding_box_[0].y, bounding_box_[2].y, &r_waypoint_rs_)
{
    // if the mobility model is created from multiple threads, move the seeding
    // of r_waypoint_rs_ outside of the parallel code (see Decoder class)
    if ( !r_waypoint_rs_.Seeded() )
    {
        r_waypoint_rs_.SetSeed(RNGMgr(network_idx).GetSeed(RS_MOBILITY));
    }
}

MobilityModel::~MobilityModel()
{
}

//* public interface to update the position based on this mobility model
void MobilityModel::GetPosition(Point& position)
{

    time_now_ = Simulator::Now();

    if ( running_ )
    {
        //1.) ..first check if mobility model needs to be updated based on time and position
        if ( CheckTriggers( position ))
        {
            // update the velocity_ vector
            UpdateMovement( position );
        }

        //2.) then update the location
        CalculatePosition( position );
    }
}


double MobilityModel::GetVelocity() {
    double velocity = sqrt( velocity_.x * velocity_.x +
                            velocity_.y * velocity_.y +
                            velocity_.z * velocity_.z );

    if( velocity > 0 && round(velocity) != round(speed_ms_))
    {
        std::cout << "speed_mps_ is " << speed_ms_ << " and velocity is " << velocity << std::endl;
        assert( velocity == speed_ms_);
    }
    return velocity;
}

void MobilityModel::Start()
{
    running_ = true;
}

void MobilityModel::Stop()
{
    running_ = false;
}

//* this method recalculates the position using the direction_ vector
void MobilityModel::CalculatePosition(Point& position)
{

    double time_delta = time_now_ - last_move_time_;
    position = position + (velocity_ * time_delta);
    last_move_time_ = time_now_;
}


bool MobilityModel::CheckTriggers(Point& position)
{
    if ( time_now_ >= next_update_time_)
    {
        return true;
    }

    // else if, position is outside the bounding box,
    // consider sending UE back to where it came etc.

    return false;
}

void MobilityModel::UpdateMovement(Point& position)
{
    if (running_)
    {

        // generate random point in bounding box
        Point new_end_position   = generate_point_();

        // determine velocity vector and how long it takes to reach the new point
        double delta_time = Distance(new_end_position, position) / speed_ms_;
        velocity_ = (new_end_position - position) / delta_time;
        next_update_time_ = time_now_ + delta_time;
    }
}


/**Create a new mobility model object of the same type.
 * The state of the cloned object is reset. Initialize must be called to use the channel.
 */
Ptr<IMobilityModel> MobilityModel::Clone() const
{
    return ::Create< MobilityModel >(*this);
}
