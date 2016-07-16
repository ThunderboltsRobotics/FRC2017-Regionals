package org.firstinspires.frc.framework.granulation;

/**
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-16/00
 */
public enum JaguarCANMessageIDs {
	setPercent(33685824), setCurrent(33687040), setSpeed(33689088), setPosition(33690048), setVoltage(33687936);

	private final int value;

	private JaguarCANMessageIDs(int i) {
		value = i;
	}

	public int getValue() {
		return value;
	}
}
