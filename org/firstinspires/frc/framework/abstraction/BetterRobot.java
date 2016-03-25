package org.firstinspires.frc.framework.abstraction;

import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tInstances;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tResourceType;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * Clutter-free replacement for RobotBase from WPILibJ v2016.0.0.
 * @author FRC 4739 Thunderbolts Robotics
 */
public class BetterRobot extends CustomRobotBase {
	private boolean m_disabledInitialized;
	private boolean m_autonomousInitialized;
	private boolean m_teleopInitialized;
	private boolean m_testInitialized;

	public BetterRobot() {
		// set status for initialization of disabled, autonomous, and teleop code.
		m_disabledInitialized = false;
		m_autonomousInitialized = false;
		m_teleopInitialized = false;
		m_testInitialized = false;
	}

	public void startCompetition() {
		UsageReporting.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Iterative);

		robotInit();

		// Tell the DS that the robot is ready to be enabled
		FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramStarting();

		// loop forever, calling the appropriate mode-dependent function
		LiveWindow.setEnabled(false);
		try {
			while (true) {
				// Call the appropriate function depending upon the current robot mode
				if (!DS.isEnabled()) {
					// call DisabledInit() if we are now just entering disabled mode from
					// either a different mode or from power-on
					if (!m_disabledInitialized) {
						LiveWindow.setEnabled(false);
						disabledInit();
						m_disabledInitialized = true;
						// reset the initialization flags for the other modes
						m_autonomousInitialized = false;
						m_teleopInitialized = false;
						m_testInitialized = false;
					}
					if (nextPeriodReady()) {
						FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramDisabled();
						disabledPeriodic();
					}
				} else if (DS.isTest()) {
					// call TestInit() if we are now just entering test mode from either
					// a different mode or from power-on
					if (!m_testInitialized) {
						LiveWindow.setEnabled(true);
						testInit();
						m_testInitialized = true;
						m_autonomousInitialized = false;
						m_teleopInitialized = false;
						m_disabledInitialized = false;
					}
					if (nextPeriodReady()) {
						FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramTest();
						testPeriodic();
					}
				} else if (DS.isAutonomous()) {
					// call Autonomous_Init() if this is the first time
					// we've entered autonomous_mode
					if (!m_autonomousInitialized) {
						LiveWindow.setEnabled(false);
						// KBS NOTE: old code reset all PWMs and relays to "safe values"
						// whenever entering autonomous mode, before calling
						// "Autonomous_Init()"
						autonomousInit();
						m_autonomousInitialized = true;
						m_testInitialized = false;
						m_teleopInitialized = false;
						m_disabledInitialized = false;
					}
					if (nextPeriodReady()) {
						FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramAutonomous();
						autonomousPeriodic();
					}
				} else {
					// call Teleop_Init() if this is the first time
					// we've entered teleop_mode
					if (!m_teleopInitialized) {
						LiveWindow.setEnabled(false);
						teleopInit();
						m_teleopInitialized = true;
						m_testInitialized = false;
						m_autonomousInitialized = false;
						m_disabledInitialized = false;
					}
					if (nextPeriodReady()) {
						FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramTeleop();
						teleopPeriodic();
					}
				}
				DS.waitForData();
			}
		} catch (Throwable t) {
			t.printStackTrace();
		}
	}

	/**
	 * Determine if the appropriate next periodic function should be called. Call
	 * the periodic functions whenever a packet is received from the Driver
	 * Station, or about every 20ms.
	 */
	private boolean nextPeriodReady() {
		return DS.isNewControlData();
	}

	public void robotInit() {}
	public void disabledInit() {}
	public void autonomousInit() {}
	public void teleopInit() {}
	public void testInit() {}
	public void disabledPeriodic() {}
	public void autonomousPeriodic() {}
	public void teleopPeriodic() {}
	public void testPeriodic() {}
}
