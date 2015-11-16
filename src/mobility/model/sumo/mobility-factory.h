#ifndef _MODELS_MOBILITY_MOBILITY_FACTORY_H_
#define _MODELS_MOBILITY_MOBILITY_FACTORY_H_

#include "simcore/object-factory.h"
#include "basic-models/mobility/interfaces/mobility-model-interface.h"
#include "basic-models/helpers/antenna-set-deployment-helper.h"
#include "io/parameters.h"
#include "basic-models/mobility/mobility-types.h"


/** Fast fading channel types. */



/**Factory for channel implementations.
 * \ingroup ChannelImplementations
 */
class MobilityFactory : public ObjectFactory<MobilityFactory, int, IMobilityModel>
{

public:
	MobilityFactory(const Parameters& p,
					const AntennaSetDeploymentHelper deployment, const int network_idx);

    ~MobilityFactory();
};

#endif /*_MOBILITY_FACTORY_H_*/