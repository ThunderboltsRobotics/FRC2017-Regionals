package org.firstinspires.frc.framework.software;

import edu.wpi.first.wpilibj.Joystick;
import org.firstinspires.frc.framework.abstraction.DSGamepadIndex;

import java.util.Objects;

/**
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/00
 */
public class HIDControllerLayout {
	public enum POVDirection {
		CENTER(-1), U(0), UR(45), R(90), DR(135), D(180), DL(225), L(270), UL(315);

		private final int value;
		POVDirection(int i) {
			value = i;
		}
		public int getValue() {
			return value;
		}

		public static POVDirection parseInt(int i) {
			switch (i) {
				case -1:
				default:
					return POVDirection.CENTER;
				case 0:
					return POVDirection.U;
				case 45:
					return POVDirection.UR;
				case 90:
					return POVDirection.R;
				case 135:
					return POVDirection.DR;
				case 180:
					return POVDirection.D;
				case 225:
					return POVDirection.DL;
				case 270:
					return POVDirection.L;
				case 315:
					return POVDirection.UL;
			}
		}
	}

	protected final DSGamepadIndex index;
	protected final Joystick rawJoystick;

	protected HIDControllerLayout(DSGamepadIndex i) {
		index = i;
		rawJoystick = new Joystick(i.getIndex());
		System.out.println("Joystick name (string): " + rawJoystick.getName());
	}

	public boolean isConnected() {
		return !Objects.equals(rawJoystick.getName(), "DISCONNECTED");
	}
}
