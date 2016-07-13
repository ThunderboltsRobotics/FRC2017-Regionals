package org.firstinspires.frc.framework.abstraction;

public class RioCANID {
	private static final int MAX_ID = 255;
	private int id;

	private static final class NonexistantRioCANIDException extends ArrayIndexOutOfBoundsException {
		private static final String ERROR_TEXT = "CAN IDs on the Rio go from 0 to " + MAX_ID;
		NonexistantRioCANIDException() {
			super(ERROR_TEXT);
		}
		NonexistantRioCANIDException(int i) {
			super(ERROR_TEXT + ", " + i + " given");
		}
	}

	public RioCANID(int i) {
		if (i < 0 || MAX_ID < i) {
			throw new NonexistantRioCANIDException(i);
		}
		id = i;
	}

	public int getIDNumber() {
		return id;
	}
}
