package org.firstinspires.frc._4739.phases;

import org.firstinspires.frc._4739.HW;
import org.firstinspires.frc.framework.software.MatchPhaseRoutine;
import org.firstinspires.frc.framework.software.hid_controller_layouts.XBOX360Controller;

import static org.firstinspires.frc.framework.software.hid_controller_layouts.XBOX360Controller.Axes.*;

/**
 * Pre-regional code (teleop).
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2017-02-19/00
 */
@SuppressWarnings("unused")
public class Teleop extends MatchPhaseRoutine {
	private static final double DEADZONE_PAIR = 0.2;
	private static final double DEADZONE_STICK = 0.1;
	private static final double DEADZONE_TRIGGER = 0.05;
	
	public void start() {}

	public void loop() {
		double turnCWPower = a(RT);
		if (turnCWPower > DEADZONE_TRIGGER) {
			HW.Drive.turnCW(turnCWPower);
			return;
		}
		double turnCCWPower = a(LT);
		if (turnCWPower > DEADZONE_TRIGGER) {
			HW.Drive.turnCW(turnCWPower);
			return;
		}
		
		double drivePower = a(LY);
		if (drivePower > DEADZONE_STICK) {
			HW.Drive.forward(drivePower);
		} else if (drivePower < -DEADZONE_STICK) {
			HW.Drive.backward(drivePower);
		} else {
			HW.Drive.stopDrive();
		}
		
		double strafePower = a(RX);
		if (strafePower > DEADZONE_STICK) {
			HW.Drive.strafeRight(strafePower);
		} else if (drivePower < -DEADZONE_STICK) {
			HW.Drive.strafeLeft(strafePower);
		} else {
			HW.Drive.stopStrafe();
		}
	}
	
	private double a(XBOX360Controller.Axes a) {
		return HW.gamepad1.getAxis(a);
	}
}
