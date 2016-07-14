package org.firstinspires.frc._4739.phases;

import org.firstinspires.frc._4739.HW;
import org.firstinspires.frc._4739.RobotMain;
import org.firstinspires.frc.framework.software.MatchPhaseRoutine;

/**
 * 4739's autonomous routine for the 2016 game (FIRST Stronghold).
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/00
 */
@SuppressWarnings("unused")
public class Auto extends MatchPhaseRoutine {
	private static final double FORWARD_SPEED = 0.4;
	private static final int PHASE_1_END = 30;

	public void start() {}

	public void loop() {
		if (RobotMain.DSInstance.getMatchTime() < PHASE_1_END) {
			//Phase 1: drive forward
			HW.Motors.Drive.drive(FORWARD_SPEED, FORWARD_SPEED);
		}
	}
}
