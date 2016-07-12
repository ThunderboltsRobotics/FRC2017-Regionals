package org.firstinspires.frc.framework.software.hid_controller_layouts;

import org.firstinspires.frc.framework.abstraction.DSGamepadIndex;
import org.firstinspires.frc.framework.software.HIDControllerLayout;

/**
 * Official controller for Microsoft's social game console, the XBOX 360. Commonly used by FRC teams.
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/00
 */
@SuppressWarnings("unused")
public class XBOX360Controller extends HIDControllerLayout {
	private class GuideButtonDisabledByWindowsException extends UnsupportedOperationException {
		GuideButtonDisabledByWindowsException() {
			super("As the official driver station only runs on Windows, where the Guide button is disabled, the Guide button is unsupported");
		}
	}

	public enum Axes {
		LX(0), LY(1), RX(4), RY(5),
		LT(2), RT(3);

		private final int value;
		Axes(int i) {
			value = i;
		}
		public int getValue() {
			return value;
		}
	}
	public enum Buttons {
		A(1), B(2), X(3), Y(4),
		LB(5), RB(6),
		LS(9), RS(10),
		BACK(7), START(8), GUIDE(11);

		private final int value;
		Buttons(int i) {
			value = i;
		}
		public int getValue() {
			return value;
		}
	}

	public XBOX360Controller(DSGamepadIndex i) {
		super(i);
	}

	/**
	 * Get the position or slider value of some axis on this controller.
	 * Note: the Analog Sticks have (1.0, 1.0) in the bottom-right corner.
	 * Note: the Triggers rest at 0.5 and go to 1.0 {???}
	 * @param a Axis to return the value of
	 * @return Value of axis a
	 */
	public double getAxis(Axes a) {
		return rawJoystick.getRawAxis(a.getValue());
	}

	/**
	 * Get the pressed state of some button on this controller.
	 * Note: the Guide button is disabled, and will remain disabled until either FIRST's DS supports it or they officially endorse another DS.
	 * @param b Button to return the state of
	 * @return True if pressed, false if not pressed or exception thrown
	 */
	public boolean getButton(Buttons b) {
		if (b == Buttons.GUIDE) {
			throw new GuideButtonDisabledByWindowsException();
		}
		return rawJoystick.getRawButton(b.getValue());
	}

	/**
	 * Get the index (assigned by the DS) of this controller.
	 * @return Index of this controller
	 */
	public DSGamepadIndex getIndex() {
		return index;
	}

	public POVDirection getPOVDirection() {
		return POVDirection.parseInt(rawJoystick.getPOV());
	}
}
