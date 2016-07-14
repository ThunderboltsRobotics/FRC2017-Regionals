package org.firstinspires.frc._4739;

import org.firstinspires.frc._4739.phases.*;
import org.firstinspires.frc.framework.granulation.BetterRobot;
import org.firstinspires.frc.framework.software.MatchPhaseRoutine;
import org.firstinspires.frc.framework.software.hid_controller_layouts.XBOX360Controller;

/**
 * Main class
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/00
 */
public class RobotMain extends BetterRobot {
	private final MatchPhaseRoutine disabled = new Disabled();
	private final MatchPhaseRoutine auto = new Auto();
	private final MatchPhaseRoutine teleop = new Teleop();
	private final MatchPhaseRoutine vision_teleop = new VisionTeleop();
	private final MatchPhaseRoutine test = new TestTeleop();

	public void initialize() {
		HW.initialize();
		disabled.start();
	}
	public void autoStart() {
		auto.start();
	}
	public void autoLoop() {
		auto.loop();
	}
	public void disabledStart() {
		disabled.start();
	}
	public void disabledLoop() {
		disabled.loop();
	}
	public void teleopStart() {
		teleop.start();
		vision_teleop.start();
	}
	public void teleopLoop() {
		if (HW.gamepad1.getButton(XBOX360Controller.Buttons.A)) {
			vision_teleop.loop();
		} else {
			teleop.loop();
		}
	}
	public void testStart() {
		test.start();
	}
	public void testLoop() {
		test.loop();
	}
}
