#pragma once

enum Command {
	COMMAND_NONE       = 0x113B3759,
    COMMAND_GET_INFO   = 0x21EC18D4,
    COMMAND_GET_STATE  = 0x3C1D2EF4,
    COMMAND_SET_PARAMS = 0x4B723BFF
};

struct SetParams {
	uint8_t p1RelayStates;
	uint8_t p2RelayStates;
	uint8_t adib1RelayState;
	uint8_t adib2RelayState;
	uint8_t extRelayState;
};

struct Request {
	uint32_t command;

    union {
    	SetParams setParams;
    };
};

struct Response {
	uint32_t command;

    union {
        struct {
        	uint16_t moduleType;
            uint8_t firmwareMajorVersion;
            uint8_t firmwareMinorVersion;
            uint32_t idw0;
            uint32_t idw1;
            uint32_t idw2;
        } getInfo;

        struct {
            float cjTemp;
        } getState;

        struct {
            uint8_t result; // 1 - success, 0 - failure
        } setParams;
    };
};

