package org.firstinspires.frc.framework;

/**
 * Things that could be used anywhere.
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-19/00
 */
public class General {
	/**
	 * Joins together the elements of a String[] (in order) into one String, with an optional separator to be inserted in-between them.
	 * @param separator (usually short) string to insert between strings in the list
	 * @param stringList List of strings to 'string' together
	 * @return Constructed string
	 */
	public static String implodeStringArray(String[] stringList, String separator) {
		String toReturn = stringList[0];
		for (int i = 1; i < stringList.length; i++) {
			toReturn += separator + stringList[i];
		}
		return toReturn;
	}

	/**
	 * Calls implodeStringArray() with a default separator of ", " (comma, space) between strings.
	 */
	public static String implodeStringArray(String[] s) {
		return implodeStringArray(s, ", ");
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
