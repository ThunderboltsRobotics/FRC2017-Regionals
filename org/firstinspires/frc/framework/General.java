package org.firstinspires.frc.framework;

/**
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-19/00
 */
public class General {
	public static String implodeStringArray(String separator, String[] strings) {
		String toReturn = strings[0];
		for (int i = 1; i < strings.length; i++) {
			toReturn += separator + strings[i];
		}
		return toReturn;
	}
	public static String implodeStringArray(String[] s) {
		return implodeStringArray(", ", s);
	}
}
