package org.firstinspires.frc.framework.hardware;

/**
 * Unfinished interface for LED strips, rings or other arrays wired to the RoboRIO via PWM.
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-14/00
 */
public class PWMLEDArray {
	private final RioHWPort port;
	public PWMLEDArray(RioHWPort p) {
		if (p.getType() != RioHWPort.PortType.PWM) {
			throw new RioHWPort.MismatchedRioPortException(RioHWPort.PortType.PWM, p.getType());
		}
		port = p;
	}

	public void set(double d) {
		//TODO
		System.out.println("new PWMLEDArray(" + port.toString() + ").set(" + d + ") called");
	}
}
