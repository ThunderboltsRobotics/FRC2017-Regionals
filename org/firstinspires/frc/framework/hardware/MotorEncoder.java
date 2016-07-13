package org.firstinspires.frc.framework.hardware;

/**
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-08/00
 */
public class MotorEncoder {
	// TODO class for type
	private final int port;

	public MotorEncoder(int initPort) {
		port = initPort;
	}

	public int getPort() {
		return port;
	}
}
