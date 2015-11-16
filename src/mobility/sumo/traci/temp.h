vechile TraCIScenarioManager::getCurrentStates(std::string vehicleId){
	Vechile v;

	std::string objectId = vehicleId;
	uint8_t variable1 = VAR_POSITION;
	uint8_t variable2 = VAR_ROAD_ID;
	uint8_t variable3 = VAR_SPEED;
	uint8_t variable4 = VAR_ANGLE;
	uint8_t variable5 = VAR_SIGNALS;

	TraCIBuffer buf = connection->query(CMD_GET_VEHICLE_VARIABLE, TraCIBuffer() << objectId << variableNumber << variable1 << variable2 << variable3 << variable4 << variable5);
	
	std::string edge;
	int signals;
	uint8_t variable1_resp; buf >> variable1_resp;
	
	while(!buf.eof()){
		if (variable1_resp == VAR_POSITION) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == POSITION_2D);
			buf >> v.px;
			buf >> v.py;
		} else if (variable1_resp == VAR_ROAD_ID) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_STRING);
			buf >> edge;
		} else if (variable1_resp == VAR_SPEED) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_DOUBLE);
			buf >> v.speed;
		} else if (variable1_resp == VAR_ANGLE) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_DOUBLE);
			buf >> v.angle_traci;
		} else if (variable1_resp == VAR_SIGNALS) {
			uint8_t varType; buf >> varType;
			ASSERT(varType == TYPE_INTEGER);
			buf >> signals;
		} else {
			opp_error("Received unhandled vehicle subscription result");
		}
	}

	v.x=px;
	v.y=py;
	v.z=0.0;
	v.velocity=speed;
	v.angle=angle_traci;
	int int_id =atoi(objectId.c_str());

	ASSERT(buf.eof());
	return v;
}

		vechile getCurrentStates(std::string vehicleId);	/** get current states of the subscribed cars */