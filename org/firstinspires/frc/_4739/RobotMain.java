package org.firstinspires.frc._4739;

import org.firstinspires.frc._4739.phases.*;
import org.firstinspires.frc.framework.granulation.BetterRobot;
import org.firstinspires.frc.framework.software.MatchPhaseRoutine;

import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * Main class (stock control system)
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-08-22/00
 */
public class RobotMain extends IterativeRobot {
	private final MatchPhaseRoutine disabled = new Disabled();
	private final MatchPhaseRoutine teleop = new Teleop();

	public void robotInit() {
		HW.initialize();
		disabled.start();
	}
	public void disabledInit() {
		disabled.start();
	}
	public void disabledPeriodic() {
		disabled.loop();
	}
	public void teleopInit() {
		teleop.start();
	}
	public void teleopPeriodic() {
		teleop.loop();
	}
}
