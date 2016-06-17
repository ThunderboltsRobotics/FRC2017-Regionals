package org.firstinspires.frc._4739;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.firstinspires.frc.framework.abstraction.BetterRobot;
import org.firstinspires.frc.framework.abstraction.MatchPhaseRoutine;

/**
 * Main class
 * @author FRC 4739 Thunderbolts Robotics
 */
public class RobotMain extends BetterRobot {
	private MatchPhaseRoutine disabled = new Disabled();
	private MatchPhaseRoutine auto = new Auto();
	private MatchPhaseRoutine teleop = new Teleop();
	private MatchPhaseRoutine vision_teleop = new VisionTeleop();
//	private MatchPhaseRoutine test = new MyTestRoutine();

	public void robotInit() { Hardware.init(); }
	public void autonomousInit() { auto.start(); }
	public void autonomousPeriodic() { auto.tick(); }
	public void disabledInit() { disabled.start(); }
	public void disabledPeriodic() { disabled.tick(); }
	public void teleopInit() { teleop.start(); }
	public void teleopPeriodic() {
		if (SmartDashboard.getBoolean("vision")) {
			visionteleop.tick();
		} else {
			teleop.tick();
		}
	}
//	public void testInit() { test.start(); }
//	public void testPeriodic() { test.tick(); }
}
