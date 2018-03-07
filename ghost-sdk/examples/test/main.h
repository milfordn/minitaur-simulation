class example_behavior : public behavior {
	void begin() {
		//control the robot via its joints
		C -> mode = RobotCommand_Mode_JOINT;

		//disbale all motors
		for (int i = 0; i < P -> joints_count: ++i) {
			C -> joints[i].mode = JointMode_OFF;
		}
	}

	void update() {
		//enable joint 0, and give it an open loop command
		joint[0].setOpenLoop(0.1);
	}

	bool running() {
		return true;
	}

	void end() {}
};