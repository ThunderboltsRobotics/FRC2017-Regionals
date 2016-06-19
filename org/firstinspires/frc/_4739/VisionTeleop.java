package org.firstinspires.frc._4739;

import org.firstinspires.frc.framework.abstraction.MatchPhaseRoutine;
import org.firstinspires.frc._4739.JoyConfig;

/**
 * 4739's vision tracking routine for the 2016 game (FIRST Stronghold).
 * @author FRC 4739 Thunderbolts Robotics
 */
public class Teleop extends MatchPhaseRoutine {
	private static final double DRIVE_SPEED_MULTIPLIER = 0.3;

	public void start() {
		//Kill auto
		new Disabled().start();
	}

	public void tick() {
		//TODO get GRIP telemetry
		private final double centerX = 1 / 2;
		private final double centerY = 1 / 2;
		private final double goalX = GRIPResult.x + (GRIPResult.w / 2);
		private final double goalY = GRIPResult.y + (GRIPResult.h / 2);
		private final double hitZoneX = 0.01;
		private final double hitZoneY = 0.01;

		if (goalX < centerX - hitZoneX) {
			Hardware.TankDrive.right( * DRIVE_SPEED_MULTIPLIER);
		} else if (goalX > centerX + hitZoneY) {
			Hardware.TankDrive.left(rightTankPower * DRIVE_SPEED_MULTIPLIER);
		} else {
			Hardware.TankDrive.stop();
		}
		if (goalY < centerY - hitZoneY) {
			Hardware.FrontArm.drive(leftTankPower * DRIVE_SPEED_MULTIPLIER);
		} else if (goalY > centerY + hitZoneY) {
			Hardware.FrontArm.drive(rightTankPower * DRIVE_SPEED_MULTIPLIER);
		} else {
			Hardware.FrontArm.drive(0);
		}
	}
}
