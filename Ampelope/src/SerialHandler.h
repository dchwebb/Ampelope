#pragma once

#include "initialisation.h"
#include "USB.h"


class SerialHandler {
public:
	SerialHandler(USBHandler& usb);
	bool Command();
	void Handler(uint8_t* data, uint32_t length);

private:
	int32_t ParseInt(const std::string cmd, const char precedingChar, int low, int high);
	float ParseFloat(const std::string cmd, const char precedingChar, float low, float high);

	// State machine for multi-stage commands
	enum class serialState {pending, dfuConfirm, calibConfirm, cancelAudioTest};
	serialState state = serialState::pending;

	bool CmdPending = false;
	std::string ComCmd;
	USBHandler* usb;
};

extern SerialHandler serial;
