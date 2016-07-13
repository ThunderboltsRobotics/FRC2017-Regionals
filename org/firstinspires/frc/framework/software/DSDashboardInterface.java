package org.firstinspires.frc.framework.software;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/00
 */
public class DSDashboardInterface {
	@SuppressWarnings("SameParameterValue")
	public static void setField(String k, boolean b) {
		SmartDashboard.putBoolean(k, b);
	}
	@SuppressWarnings("SameParameterValue")
	public static void setField(String k, Number n) {
		SmartDashboard.putNumber(k, (double) n);
	}
	@SuppressWarnings("SameParameterValue")
	public static void setField(String k, String s) {
		SmartDashboard.putString(k, s);
	}
	@SuppressWarnings("SameParameterValue")
	public static boolean getBooleanField(String k) {
		return SmartDashboard.getBoolean(k);
	}
	@SuppressWarnings("SameParameterValue")
	public static double getNumberField(String k) {
		return SmartDashboard.getNumber(k);
	}
	@SuppressWarnings("SameParameterValue")
	public static String getStringField(String k) {
		return SmartDashboard.getString(k);
	}
}
