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
		private final double centerX = 1/2;
		private final double centerY = 1/2;
		private final double goalX = GRIPResult.x + (GRIPResult.w / 2);
		private final double goalY = GRIPResult.y + (GRIPResult.h / 2);
		//TODO
		if () {
			Hardware.TankDrive.left(leftTankPower * DRIVE_SPEED_MULTIPLIER);
		} else if () {
			Hardware.TankDrive.right(rightTankPower * DRIVE_SPEED_MULTIPLIER);
		}

		//Front arm
		frontArmPower = JoyConfig.Joy1.getLeftTrigger();
		if (frontArmPower > JoyConfig.Joy1.TRIGGGER_DEADZONE) {
			Hardware.FrontArm.drive(frontArmPower);
		} else {
			frontArmPower = -JoyConfig.Joy1.getRightTrigger();
			if (frontArmPower < -JoyConfig.Joy1.TRIGGGER_DEADZONE) {
				Hardware.FrontArm.drive(frontArmPower);
			} else {
				Hardware.FrontArm.drive(0);
			}
		}
	}
}
