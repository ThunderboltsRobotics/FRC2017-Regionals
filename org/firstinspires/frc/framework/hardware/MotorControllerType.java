package org.firstinspires.frc.framework.hardware;

/**
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/00
 */
public enum MotorControllerType {
	Jaguar(true),
	SD540(false),
	Spark(false),
	Talon(false), TalonSR(false), TalonSRX(true),
	Victor(false), VictorSP(false);

	private final boolean isCANEnabled;
	MotorControllerType(boolean isCANEnabled) {
		this.isCANEnabled = isCANEnabled;
	}
	public boolean getIsCANEnabled() {
		return isCANEnabled;
	}
}
