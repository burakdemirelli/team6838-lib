package com.team6838.lib.drivers;

public interface CKIMU {

	double getFusedHeading();

	double getRawYawDegrees();

	double getPitch();

	double getRoll();

	boolean isPresent();

	boolean reset();
}
