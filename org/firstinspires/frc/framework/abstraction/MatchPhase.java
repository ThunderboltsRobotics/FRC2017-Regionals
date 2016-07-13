package org.firstinspires.frc.framework.abstraction;

/**
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/00
 */
public enum MatchPhase {
	Disabled(0), Auto(1), Teleop(2), Test(3);

	private final int value;
	MatchPhase(int i) {
		value = i;
	}
	public int getValue() {
		return value;
	}
}
