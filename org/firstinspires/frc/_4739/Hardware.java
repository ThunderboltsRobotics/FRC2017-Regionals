package org.firstinspires.frc._4739;

import org.firstinspires.frc._4739.RioPortConfig;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;

/**
 * 4739's hardware setup for the 2016 game (FIRST Stronghold).
 * @author FRC 4739 Thunderbolts Robotics
 */
public class Hardware {
	private static final double RIGHT_SPEED_MULTIPLIER = 0.95; //Our Talons aren't calibrated or something

	public static final Talon[] talons = new Talon[10];
	public static final Victor[] victors = new Victor[10];

	public static final void init() {
		talons[RioPortConfig.leftDriveA] = new Talon(RioPortConfig.leftDriveA);
		talons[RioPortConfig.leftDriveB] = new Talon(RioPortConfig.leftDriveB);
		talons[RioPortConfig.rightDriveA] = new Talon(RioPortConfig.rightDriveA);
		talons[RioPortConfig.rightDriveB] = new Talon(RioPortConfig.rightDriveB);
		victors[RioPortConfig.frontArm] = new Victor(RioPortConfig.frontArm);
	}

	public static final class TankDrive {
		public static final void left(double d) {
			talons[RioPortConfig.leftDriveA].set(d * RIGHT_SPEED_MULTIPLIER);
			talons[RioPortConfig.leftDriveB].set(d * RIGHT_SPEED_MULTIPLIER);
		}
		public static final void right(double d) {
			talons[RioPortConfig.rightDriveA].set(-d);
			talons[RioPortConfig.rightDriveB].set(-d);
		}
	}

	public static final class FrontArm {
		public static final void drive(double d) {
			victors[RioPortConfig.frontArm].set(d);
		}
	}
}
