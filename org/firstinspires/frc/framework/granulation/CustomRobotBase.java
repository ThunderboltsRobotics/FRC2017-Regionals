package org.firstinspires.frc.framework.granulation;

import java.net.URL;
import java.util.Enumeration;
import java.util.jar.Manifest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.HLUsageReporting;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary;
import edu.wpi.first.wpilibj.internal.HardwareHLUsageReporting;
import edu.wpi.first.wpilibj.internal.HardwareTimer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * Clutter-free replacement for RobotBase from WPILibJ v2016.0.0.
 * RobotBase.java (decompiled) totals to < 200 lines. This file is < 100 lines.
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-13/00
 */
abstract class CustomRobotBase extends RobotBase {
	final DriverStation DS = DriverStation.getInstance();

	CustomRobotBase() {
		NetworkTable.setNetworkIdentity("Robot");
		NetworkTable.setPersistentFilename("/home/lvuser/networktables.ini");
		NetworkTable.setServerMode();
		NetworkTable.getTable("");
	}

	public static void main(String args[]) {
		String robotName = null;
		CustomRobotBase robot;

		//TODO check if these lines are required
		FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationReserve();
		Timer.SetImplementation(new HardwareTimer());
		HLUsageReporting.SetImplementation(new HardwareHLUsageReporting());
		RobotState.SetImplementation(DriverStation.getInstance());

		try {
			Enumeration<URL> resources = RobotBase.class.getClassLoader().getResources("META-INF/MANIFEST.MF");
			while (resources.hasMoreElements()) {
				try {
					robotName = new Manifest(resources.nextElement().openStream()).getMainAttributes().getValue("Robot-Class");
				} catch (Throwable t) {
					t.printStackTrace();
				}
			}
		} catch (Throwable t) {
			t.printStackTrace();
		}

		try {
			robot = (CustomRobotBase) Class.forName(robotName).newInstance();
			try {
				robot.startCompetition();
			} catch (Throwable t) {
				t.printStackTrace();
			}
		} catch (Throwable t) {
			t.printStackTrace();
		}
	}

	public abstract void startCompetition();
}
