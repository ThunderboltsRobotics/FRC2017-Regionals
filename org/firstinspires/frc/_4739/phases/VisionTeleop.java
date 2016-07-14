package org.firstinspires.frc._4739.phases;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import org.firstinspires.frc._4739.HW;
import org.firstinspires.frc.framework.software.MatchPhaseRoutine;

/**
 * 4739's vision tracking routine for the 2016 game (FIRST Stronghold).
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/00
 */
@SuppressWarnings("unused")
public class VisionTeleop extends MatchPhaseRoutine {
	private static final double CENTER_X = 1 / 2;
	private static final double CENTER_Y = 2 / 3;
	private static final double HITZONE_X = 0.01;
	private static final double HIT_ZONE_Y = 0.01;
	private static final double MAX_SPEED_HORIZONTAL = 0.3;
	private static final double MAX_SPEED_VERTICAL = 1;
	private final double[] defaultGoalResult = {
			0,
			0,
			0,
			0,
			0
	};

	public void start() {
		//Kill auto
		new Disabled().start();
	}

	public void loop() {
		NetworkTable GRIPResultTable = NetworkTable.getTable("GRIP");
		double[] GRIPResult = GRIPResultTable.getNumberArray("", defaultGoalResult);
		//TODO check GRIP outputs
		final double goalX = GRIPResult[3] + (GRIPResult[2] / 2); //x + w/2
		final double goalY = GRIPResult[4] + (GRIPResult[1] / 2); //y + h/2

		if (goalX < CENTER_X - HITZONE_X) {
			HW.Motors.Drive.right(Math.sqrt(CENTER_X - goalX) * MAX_SPEED_HORIZONTAL);
		} else if (goalX > CENTER_X + HIT_ZONE_Y) {
			HW.Motors.Drive.left(Math.sqrt(goalX - CENTER_X) * MAX_SPEED_HORIZONTAL);
		} else {
			HW.Motors.Drive.right(0);
			HW.Motors.Drive.left(0);
		}
		if (goalY < CENTER_Y - HIT_ZONE_Y) {
			HW.Motors.Shooter.relAim(Math.sqrt(goalY - CENTER_Y) * MAX_SPEED_VERTICAL);
		} else if (goalY > CENTER_Y + HIT_ZONE_Y) {
			HW.Motors.Shooter.relAim(Math.sqrt(CENTER_Y - goalY) * MAX_SPEED_VERTICAL);
		} else {
			HW.Motors.Shooter.relAim(0);
		}
	}
}
