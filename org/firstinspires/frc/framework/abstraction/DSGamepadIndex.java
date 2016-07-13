package org.firstinspires.frc.framework.abstraction;

/**
 * @author yoshi
 * @version 2016-07-09/00
 */
public class DSGamepadIndex {
	private static final int MAX_CONTROLLERS = 6;
	private int index;

	private static final class NonexistantGamepadIndexException extends ArrayIndexOutOfBoundsException {
		private static final String ERROR_TEXT = "The DS tracks HID controllers/gamepads 1 to " + MAX_CONTROLLERS;
		NonexistantGamepadIndexException() {
			super(ERROR_TEXT);
		}
		NonexistantGamepadIndexException(int i) {
			super(ERROR_TEXT + ", " + i + " given");
		}
	}

	public DSGamepadIndex(int i) {
		if (i < 1 || MAX_CONTROLLERS < i) {
			throw new NonexistantGamepadIndexException(i);
		}
		index = i;
	}

	public int getIndex() {
		return index;
	}
}
