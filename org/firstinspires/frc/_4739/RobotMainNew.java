package org.firstinspires.frc._4739;

import org.firstinspires.frc._4739.phases.Disabled;
import org.firstinspires.frc._4739.phases.Teleop;
import org.firstinspires.frc.framework.granulation.BetterRobot;
import org.firstinspires.frc.framework.software.MatchPhaseRoutine;

/**
 * Main class (custom control system)
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-08-22/00
 */
public class RobotMainNew extends BetterRobot {
	private final MatchPhaseRoutine disabled = new Disabled();
	private final MatchPhaseRoutine teleop = new Teleop();

	public void init() {
		HW.initialize();
		disabled.start();
	}

	public void disabledStart() {
		disabled.start();
	}
	public void disabledLoop() {
		disabled.loop();
	}
	public void teleopStart() {
		teleop.start();
	}
	public void teleopLoop() {
		teleop.loop();
	}

	public void autoStart() {}
	public void autoLoop() {}
	public void testStart() {}
	public void testLoop() {}
}
