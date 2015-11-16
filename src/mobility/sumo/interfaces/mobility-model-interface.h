#ifndef _MODELS_MOBILITY_INTERFACES_MOBILITY_MODEL_INTERFACE_H_
#define _MODELS_MOBILITY_INTERFACES_MOBILITY_MODEL_INTERFACE_H_

#include "simcore/ptr.h"
#include "simcore/interfaces/referencecounter.h"
#include "math/geometry/point.h"
//#include "basic-models/mobility/mobility-factory.h"



//TRAIL [DORA_Integration:008] common interface for all mobility models
//.. currently only GetPosition(Point& position) is used.

class IMobilityModel : public virtual IReferenceCounter
{
public:

    //virtual ~IMobilityModel() = 0;

    /// Initialize method is used to adjust the mobility model per UE basis.
    //virtual void Initialize(double speed_in_ms) = 0;

    /// GetPosition method is the shared interface for all mobility models as it
    /// ..updates the given position according to the mobility model.
    virtual void GetPosition(Point& position) = 0;
    virtual double GetVelocity() = 0;

    virtual double       GetAngle() = 0;            // ADD cam traffic
    virtual double       GetAcceleration() = 0;
    virtual double       GetStaLength() = 0;
    virtual double       GetStaWidth() = 0;
    virtual int          GetLights() = 0;
    virtual std::string  GetLaneId() = 0;
    virtual std::string  GetEdgeId() = 0;
    virtual std::string  GetJunctionId() = 0;

    virtual Ptr<IMobilityModel> Clone() const = 0;

    //virtual MobilityType GetType() = 0; //added by
    virtual void SetId(int id) = 0;
    virtual int GetId() const = 0;

};

#endif /*_MODELS_MOBILITY_MOBILITY_MODEL_H_*/