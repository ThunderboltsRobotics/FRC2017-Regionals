package org.firstinspires.frc.framework.abstraction;

/**
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/00
 */
public enum RioPWMPort {
	PWM_0(0), PWM_1(1), PWM_2(2), PWM_3(3), PWM_4(4), PWM_5(5), PWM_6(6), PWM_7(7), PWM_8(8), PWM_9(9);

	private final int index;
	RioPWMPort(int i) {
		index = i;
	}
	public int getPortNumber() {
		return index;
	}
}
