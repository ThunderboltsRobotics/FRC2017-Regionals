package org.firstinspires.frc.framework.software.hid_controller_layouts;

import edu.wpi.first.wpilibj.Joystick;
import org.firstinspires.frc.framework.abstraction.DSGamepadIndex;
import org.firstinspires.frc.framework.software.HIDControllerLayout;

/**
 * Logitech's Extreme 3D Pro joystick.
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/00
 */
@SuppressWarnings("unused")
public class LogitechExtreme3DProJoystick extends HIDControllerLayout {
	public enum Axes {
		THROTTLE(3),
		X(0), Y(1), Z(2);

		private final int value;
		Axes(int i) {
			value = i;
		}
		public int getValue() {
			return value;
		}
	}
	public enum Buttons {
		TRIGGER(1),
		B2(2), B3(3), B4(4), B5(5), B6(6), B7(7), B8(8), B9(9), B10(10), B11(11), B12(12);

		private final int value;
		Buttons(int i) {
			value = i;
		}
		public int getValue() {
			return value;
		}
	}

	private DSGamepadIndex index;
	private Joystick rawJoystick;

	public LogitechExtreme3DProJoystick(DSGamepadIndex i) {
		super(i);
	}

	/**
	 * Get the position or slider value of some axis on this joystick.
	 * Note: {???}
	 * @param a Axis to return the value of
	 * @return Value of axis a
	 */
	public double getAxis(Axes a) {
		return rawJoystick.getRawAxis(a.getValue());
	}

	/**
	 * Get the pressed state of some button on this joystick.
	 * @param b Button to return the state of
	 * @return True if pressed, false if not pressed or exception thrown
	 */
	public boolean getButton(Buttons b) {
		return rawJoystick.getRawButton(b.getValue());
	}

	/**
	 * Get the index (assigned by the DS) of this joystick.
	 * @return Index of this joystick
	 */
	public DSGamepadIndex getIndex() {
		return index;
	}

	public POVDirection getPOVDirection() {
		return POVDirection.parseInt(rawJoystick.getPOV());
	}
}
