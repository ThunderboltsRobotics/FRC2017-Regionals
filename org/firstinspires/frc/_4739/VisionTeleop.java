package org.firstinspires.frc._4739;

import org.firstinspires.frc.framework.abstraction.MatchPhaseRoutine;

/**
 * 4739's vision tracking routine for the 2016 game (FIRST Stronghold).
 * @author FRC 4739 Thunderbolts Robotics
 */
public class Teleop extends MatchPhaseRoutine {
	private final double centerX = 1 / 2;
	private final double centerY = 2 / 3;
	private final double hitZoneX = 0.01;
	private final double hitZoneY = 0.01;
	private final double maxSpeedX = 0.3;
	private final double maxSpeedY = 1;
	private GRIPResult;

	public void start() {
		//Kill auto
		new Disabled().start();
	}

	public void tick() {
		//TODO get GRIP telemetry
		//GRIPResult = ;
		private final double goalX = GRIPResult.x + (GRIPResult.w / 2);
		private final double goalY = GRIPResult.y + (GRIPResult.h / 2);

		if (goalX < centerX - hitZoneX) {
			Hardware.TankDrive.right(Math.sqrt(centerX - goalX) * maxSpeedX);
		} else if (goalX > centerX + hitZoneY) {
			Hardware.TankDrive.left(Math.sqrt(goalX - centerX) * maxSpeedX);
		} else {
			Hardware.TankDrive.stop();
		}
		if (goalY < centerY - hitZoneY) {
			Hardware.FrontArm.drive(Math.sqrt(goalY - centerY) * maxSpeedY);
		} else if (goalY > centerY + hitZoneY) {
			Hardware.FrontArm.drive(Math.sqrt(centerY - goalY) * maxSpeedY);
		} else {
			Hardware.FrontArm.drive(0);
		}
	}
}
