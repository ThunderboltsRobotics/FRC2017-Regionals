package org.firstinspires.frc._4739;

import org.firstinspires.frc.framework.abstraction.MatchPhaseRoutine;

/**
 * 4739's autonomous routine for the 2016 game (FIRST Stronghold).
 * @author FRC 4739 Thunderbolts Robotics
 */
public class Auto extends MatchPhaseRoutine {
	private static final double FORWARD_SPEED = 0.4;
	private static final int PHASE_1_END = 30;
	private static final int PHASE_2_END = 150;
	private long currentTime;

	public void start() {
		currentTime = 0;
	}

	public void tick() {
		if (currentTime < PHASE_1_END) {
			//Phase 1: lower front arm
			Hardware.FrontArm.drive(1);
		}
		if (currentTime < PHASE_2_END) {
			//Phase 1 and 2: drive forward
			Hardware.TankDrive.left(FORWARD_SPEED);
			Hardware.TankDrive.right(FORWARD_SPEED);
		}
		currentTime++;
	}
}
