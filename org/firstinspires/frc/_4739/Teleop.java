package org.firstinspires.frc._4739;

import org.firstinspires.frc.framework.abstraction.MatchPhaseRoutine;
import org.firstinspires.frc._4739.JoyConfig;

/**
 * 4739's teleop routine for the 2016 game (FIRST Stronghold).
 * @author FRC 4739 Thunderbolts Robotics
 */
public class MatchTeleop2016 extends MatchPhaseRoutine {
	private static final double DRIVE_SPEED_MULTIPLIER = 0.3;
	private static double frontArmPower, leftTankPower, rightTankPower;

	public void start() {
		//Kill auto
		new Disabled().start();
	}

	public void tick() {
		//Drive
		leftTankPower = JoyConfig.Joy1.getLeftYAxis();
		rightTankPower = JoyConfig.Joy1.getRightYAxis();
		if (leftTankPower < -JoyConfig.Joy1.ANALOG_STICK_DEADZONE || leftTankPower > JoyConfig.Joy1.ANALOG_STICK_DEADZONE) {
			Hardware.TankDrive.left(leftTankPower * DRIVE_SPEED_MULTIPLIER);
		}
		if (rightTankPower < -JoyConfig.Joy1.ANALOG_STICK_DEADZONE || rightTankPower > JoyConfig.Joy1.ANALOG_STICK_DEADZONE) {
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
