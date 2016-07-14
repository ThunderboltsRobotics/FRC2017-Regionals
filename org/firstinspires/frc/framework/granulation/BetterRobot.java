package org.firstinspires.frc.framework.granulation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.firstinspires.frc.framework.abstraction.MatchPhase;

/**
 * Clutter-free replacement for IterativeRobot from WPILibJ v2016.0.0.
 * IterativeRobot.java (decompiled) totals to < 200 lines. This file is < 200 lines.
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/03
 */
public abstract class BetterRobot extends CustomRobotBase {
	public static final DriverStation DSInstance = DriverStation.getInstance();
	private MatchPhase lastInitialized;

	private void initializePhase(MatchPhase m) {
		LiveWindow.setEnabled(m != MatchPhase.Disabled);
		switch (m) {
			case Disabled:
				disabledStart();
				break;
			case Auto:
				autoStart();
				break;
			case Teleop:
				teleopStart();
				break;
			case Test:
				testStart();
				break;
		}
		lastInitialized = m;
	}
	private void reportCodeReady() {
		FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramStarting();
	}
	private void reportCodeRunning(MatchPhase m) {
		switch (m) {
			case Disabled:
				FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramDisabled();
				break;
			case Auto:
				FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramAutonomous();
				break;
			case Teleop:
				FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramTeleop();
				break;
			case Test:
				FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramTest();
				break;
		}
	}

	public void startCompetition() {
		initialize();
		reportCodeReady();
		while (true) try {
			if (DS.isEnabled()) {
				if (DS.isTest()) {
					if (lastInitialized != MatchPhase.Test) {
						initializePhase(MatchPhase.Test);
					}
				} else if (DS.isAutonomous()) {
					if (lastInitialized != MatchPhase.Auto) {
						initializePhase(MatchPhase.Auto);
					}
				} else {
					if (lastInitialized != MatchPhase.Teleop) {
						initializePhase(MatchPhase.Teleop);
					}
				}
			} else {
				if (lastInitialized != MatchPhase.Disabled) {
					initializePhase(MatchPhase.Disabled);
				}
			}
			if (DS.isNewControlData()) {
				reportCodeRunning(lastInitialized);
				switch (lastInitialized) {
					case Disabled:
						disabledLoop();
						break;
					case Auto:
						autoLoop();
						break;
					case Teleop:
						teleopLoop();
						break;
					case Test:
						testLoop();
						break;
				}
			}
			DS.waitForData();
		} catch (Throwable t) {
			//Display error in DS
			System.out.println("Error caught in a routine running during the " +
					(DS.isEnabled() ? (
							(DS.isTest() ? "Test" : (
									(DS.isAutonomous() ? "Auto" : "Teleop")
							))
					) : "Disabled")
					+ " phase");
			t.printStackTrace();
			break;
		}
		//Graceful shutdown
		System.exit(0);
	}

	/**
	 * Called once when the robot is first powered on.
	 * To be overwritten.
	 */
	public abstract void initialize();

	/**
	 * Called when the Disabled phase is started (incl. when first powered on).
	 * To be overwritten.
	 */
	public abstract void disabledStart();
	/**
	 * Called at 50Hz during the Disabled phase.
	 * To be overwritten.
	 */
	public abstract void disabledLoop();

	/**
	 * Called when the Autonomous phase is started (incl. when first powered on).
	 * To be overwritten.
	 */
	public abstract void autoStart();
	/**
	 * Called at 50Hz during the Autonomous phase.
	 * To be overwritten.
	 */
	public abstract void autoLoop();
	/**
	 * Called when the Teleop phase is started (incl. when first powered on).
	 * To be overwritten.
	 */
	public abstract void teleopStart();
	/**
	 * Called at 50Hz during the Teleop phase.
	 * To be overwritten.
	 */
	public abstract void teleopLoop();

	/**
	 * Called when the Test phase is started (incl. when first powered on).
	 * To be overwritten.
	 */
	public abstract void testStart();
	/**
	 * Called at 50Hz during the Test phase.
	 * To be overwritten.
	 */
	public abstract void testLoop();
}
