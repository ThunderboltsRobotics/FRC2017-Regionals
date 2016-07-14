package org.firstinspires.frc._4739.phases;

import org.firstinspires.frc.framework.software.DSDashboardInterface;
import org.firstinspires.frc.framework.software.MatchPhaseRoutine;

/**
 * 4739's teleop routine for the 2016 game (FIRST Stronghold).
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/00
 */
@SuppressWarnings("unused")
public class TestTeleop extends MatchPhaseRoutine {
	public void start() {
		//Kill auto
		new Disabled().start();
	}

	public void loop() {
		DSDashboardInterface.setField("Test", true);
	}
}
