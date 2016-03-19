package org.firstinspires.frc4739.framework;

public class POVConversion {
	public static enum POVDirection {
		CENTER, U, UR, R, DR, D, DL, L, UL
	}

	public static final int POV_CENTER = -1;
	public static final int POV_U = 0;
	public static final int POV_UR = 45;
	public static final int POV_R = 90;
	public static final int POV_DR = 135;
	public static final int POV_D = 180;
	public static final int POV_DL = 225;
	public static final int POV_L = 270;
	public static final int POV_UL = 315;

	public static POVDirection intToPOV(int i) {
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
