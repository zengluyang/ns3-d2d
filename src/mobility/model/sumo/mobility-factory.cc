#include "simcore/create.h"
#include "simcore/checked-enum.h"
#include "basic-models/mobility/mobility-factory.h"
#include "basic-models/mobility/mobility-model.h"

#include "basic-models/mobility/sumo-model.h"     //added by

//#include "models/global-defs.h"
#include "io/parameters.h"
#include "math/random/rng-manager.h"
#include "math/geometry/point.h"
#include <iostream>
#include <vector>


/**Create a mobility factory.
 * @param p parameters to be used to setup the factory..
 */
MobilityFactory::MobilityFactory(const Parameters& p,
		   	   	   	   	   	   	 const AntennaSetDeploymentHelper deployment, const int network_idx)
{
    //by default use the topology of bounding_box;
    std::vector<Point> box = deployment.GetDeployment(network_idx)->BoundingBox();
    double speed_in_ms = p.deployment.ue_speed/3.6;

    // if NO_MOBILITY is enabled, mobility factor returns nullptr
    Add(NO_MOBILITY, nullptr);
    std::cout<<box<<std::endl;
	  Add(BASIC, ::Create<MobilityModel>(network_idx, box, speed_in_ms));
    Add(SUMO_MODEL, ::Create<SumoModel>()); //added by wW


}

MobilityFactory::~MobilityFactory()
{
}
