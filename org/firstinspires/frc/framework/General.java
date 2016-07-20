package org.firstinspires.frc.framework;

/**
 * Things that could be used anywhere.
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

	public static final class ExtraParametersNotApplicableException extends IllegalArgumentException {
		public ExtraParametersNotApplicableException(String methodName, String[] parameters) {
			super("The extra argument" + (parameters.length > 1 ? "s" : "") + " for " + methodName + "(..." + implodeStringArray(parameters) + ") " + (parameters.length > 1 ? "are" : "is") + "n't used there");
		}
	}

	/**
	 * If you've ever pressed F3 (Eclipse) or <Ctrl>B (IDEA) on something robot-related that you didn't write, you know what this is for.
	 * @author FRC 4739 Thunderbolts Robotics
	 * @version 2016-07-14/00
	 */
	public static class WPILibJExemplifiesPoorProgrammingException extends Exception {
		public WPILibJExemplifiesPoorProgrammingException(String s) {
			super(s);
		}
	}
}
