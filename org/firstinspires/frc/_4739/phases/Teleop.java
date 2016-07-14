package org.firstinspires.frc._4739.phases;

import org.firstinspires.frc._4739.HW;
import org.firstinspires.frc.framework.software.DSDashboardInterface;
import org.firstinspires.frc.framework.software.MatchPhaseRoutine;
import org.firstinspires.frc.framework.software.hid_controller_layouts.XBOX360Controller.Axes;
import org.firstinspires.frc.framework.software.hid_controller_layouts.XBOX360Controller.Buttons;

/**
 * 4739's teleop routine for the 2016 game (FIRST Stronghold).
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/00
 */
@SuppressWarnings("unused")
public class Teleop extends MatchPhaseRoutine {
	private static final double DRIVE_SPEED_MULTIPLIER = 0.7;
	private static final double ANALOG_STICK_DEADZONE = 0.05;
	private static final double TRIGGER_DEADZONE = 0.1;
	private static final int SHOOTER_INCL_UP = 0;
	private static final int SHOOTER_INCL_DIAG = 30;
	private static final int SHOOTER_INCL_FLAT = 60;
	private static final int SHOOTER_INCL_REST = 90;

	public void start() {
		//Kill auto
		new Disabled().start();
	}

	public void loop() {
		//Drive
		double leftTankPower = HW.gamepad1.getAxis(Axes.LY);
		double rightTankPower = HW.gamepad1.getAxis(Axes.RY);
		if (leftTankPower < -ANALOG_STICK_DEADZONE || leftTankPower > ANALOG_STICK_DEADZONE) {
			HW.Motors.Drive.left(leftTankPower * DRIVE_SPEED_MULTIPLIER);
		}
		if (rightTankPower < -ANALOG_STICK_DEADZONE || rightTankPower > ANALOG_STICK_DEADZONE) {
			HW.Motors.Drive.right(rightTankPower * DRIVE_SPEED_MULTIPLIER);
		}
		DSDashboardInterface.setField("leftTankPower", leftTankPower);
		DSDashboardInterface.setField("rightTankPower", rightTankPower);

		//Shooter aiming (manual)
		if (HW.gamepad2.isConnected()) {
			//Use two gamepads
			double shooterAimInclination = SHOOTER_INCL_REST;
			if (HW.gamepad2.getButton(Buttons.A)) {
				//Aim with LAnalog
				shooterAimInclination = Math.atan2(HW.gamepad2.getAxis(Axes.LY), HW.gamepad2.getAxis(Axes.LX));
				//TODO make up 0, down +-180, then take magnitude
			} else if (HW.gamepad2.getButton(Buttons.B)) {
				//Aim with D-pad
				switch (HW.gamepad2.getPOVDirection()) {
					case U:
						shooterAimInclination = SHOOTER_INCL_UP;
						break;
					case UL:
					case UR:
						shooterAimInclination = SHOOTER_INCL_DIAG;
						break;
					case L:
					case R:
						shooterAimInclination = SHOOTER_INCL_FLAT;
						break;
					default:
						shooterAimInclination = SHOOTER_INCL_REST;
				}
			}
			DSDashboardInterface.setField("shooterAimInclination", shooterAimInclination);
			HW.Motors.Shooter.aim(shooterAimInclination);

			if (HW.gamepad2.getButton(Buttons.Y)) {
				HW.Motors.Shooter.fire();
			} else if (HW.gamepad2.getButton(Buttons.X)) {
				HW.Motors.Shooter.intake();
			} else {
				HW.Motors.Shooter.stop();
			}
		} else {
			//Use only one gamepad
			if (HW.gamepad1.getAxis(Axes.RT) > TRIGGER_DEADZONE) {
				HW.Motors.Shooter.relAim(0.3);
			} else if (HW.gamepad1.getAxis(Axes.RT) > TRIGGER_DEADZONE) {
				HW.Motors.Shooter.relAim(-0.3);
			} else {
				HW.Motors.Shooter.relAim(0);
			}
			if (HW.gamepad1.getButton(Buttons.A)) {
				HW.Motors.Shooter.fire();
			} else {
				HW.Motors.Shooter.stop();
			}
		}
	}
}
