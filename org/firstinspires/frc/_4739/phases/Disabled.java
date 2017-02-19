package org.firstinspires.frc._4739.phases;

import org.firstinspires.frc._4739.HW;
import org.firstinspires.frc.framework.software.MatchPhaseRoutine;

/**
 * Pre-regional code (disabled).
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2017-02-19/00
 */
@SuppressWarnings("unused")
public class Disabled extends MatchPhaseRoutine {
	public void start() {
		HW.Drive.stopDrive();
		HW.Drive.stopStrafe();
	}

	public void loop() {
		start();
	}
}
