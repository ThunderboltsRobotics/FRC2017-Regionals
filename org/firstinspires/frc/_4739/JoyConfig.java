package org.firstinspires.frc._4739;

import org.firstinspires.frc.framework.POVConversion;
import org.firstinspires.frc.framework.joyconfiglayouts.Extreme3DProLayout;
import org.firstinspires.frc.framework.joyconfiglayouts.XBox360Layout;

import edu.wpi.first.wpilibj.Joystick;

public class JoyConfig {
	private static final Joystick joy1 = new Joystick(0);
	private static final Joystick joy2 = new Joystick(1);

	public static final class Joy1 {
		public static final double ANALOG_STICK_DEADZONE = 1/20;
		public static final double TRIGGGER_DEADZONE = 1/10;

		public static boolean getButtonA() {
			return joy1.getRawButton(XBox360Layout.BUTTON_A);
		}
		public static boolean getButtonB() {
			return joy1.getRawButton(XBox360Layout.BUTTON_B);
		}
		public static boolean getButtonBack() {
			return joy1.getRawButton(XBox360Layout.BUTTON_X);
		}
		public static boolean getButtonStart() {
			return joy1.getRawButton(XBox360Layout.BUTTON_X);
		}
		public static boolean getButtonX() {
			return joy1.getRawButton(XBox360Layout.BUTTON_X);
		}
		public static boolean getButtonY() {
			return joy1.getRawButton(XBox360Layout.BUTTON_Y);
		}
		public static boolean getLeftBumper() {
			return joy1.getRawButton(XBox360Layout.BUTTON_LB);
		}
		public static double getLeftTrigger() {
			return joy1.getRawAxis(XBox360Layout.AXIS_LT);
		}
		public static double getLeftXAxis() {
			return joy1.getRawAxis(XBox360Layout.AXIS_LS_X) * XBox360Layout.AXIS_LS_X_DIRECTION;
		}
		public static double getLeftYAxis() {
			return joy1.getRawAxis(XBox360Layout.AXIS_LS_Y) * XBox360Layout.AXIS_LS_Y_DIRECTION;
		}
		public static POVConversion.POVDirection getPOVDirection() {
			return POVConversion.intToPOV(joy1.getPOV());
		}
		public static boolean getRightBumper() {
			return joy1.getRawButton(XBox360Layout.BUTTON_RB);
		}
		public static double getRightTrigger() {
			return joy1.getRawAxis(XBox360Layout.AXIS_RT);
		}
		public static double getRightXAxis() {
			return joy1.getRawAxis(XBox360Layout.AXIS_RS_X) * XBox360Layout.AXIS_RS_X_DIRECTION;
		}
		public static double getRightYAxis() {
			return joy1.getRawAxis(XBox360Layout.AXIS_RS_Y) * XBox360Layout.AXIS_RS_Y_DIRECTION;
		}
	}

	public static final class Joy2 {
		public static final double JOYSTICK_DEADZONE = 1/20;
		public static final double ROTATION_DEADZONE = 1/20;

		public static boolean getButton(int button) {
			if (button < 1 || 12 < button) {
				throw new ArrayIndexOutOfBoundsException("Buttons on this controller go from 2 to 12, " + button + " given");
			}
			return joy2.getRawButton(button);
		}

		public static POVConversion.POVDirection getPOVDirection() {
			return POVConversion.intToPOV(joy2.getPOV());
		}
		public static double getThrottle() {
			return joy2.getRawAxis(Extreme3DProLayout.AXIS_THROTTLE) * Extreme3DProLayout.AXIS_THROTTLE_DIRECTION;
		}
		public static boolean getTrigger() {
			return joy2.getRawButton(Extreme3DProLayout.BUTTON_TRIGGER);
		}
		public static double getXAxis() {
			return joy2.getRawAxis(Extreme3DProLayout.AXIS_X) * Extreme3DProLayout.AXIS_X_DIRECTION;
		}
		public static double getYAxis() {
			return joy2.getRawAxis(Extreme3DProLayout.AXIS_Y) * Extreme3DProLayout.AXIS_Y_DIRECTION;
		}
		public static double getZAxis() {
			return joy2.getRawAxis(Extreme3DProLayout.AXIS_Z) * Extreme3DProLayout.AXIS_Z_DIRECTION;
		}
	}
}
