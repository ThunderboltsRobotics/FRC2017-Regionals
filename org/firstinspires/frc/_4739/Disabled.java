package org.firstinspires.frc._4739;

import org.firstinspires.frc.framework.abstraction.MatchPhaseRoutine;

/**
 * 4739's disabled routine for the 2016 game (FIRST Stronghold).
 * @author FRC 4739 Thunderbolts Robotics
 */
public class Disabled2016 extends MatchPhaseRoutine {
	public void start() {
		Hardware.FrontArm.drive(0);
		Hardware.TankDrive.left(0);
		Hardware.TankDrive.right(0);
	}

	public void tick(){}
}
