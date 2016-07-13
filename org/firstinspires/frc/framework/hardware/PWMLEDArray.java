package org.firstinspires.frc.framework.hardware;

import org.firstinspires.frc.framework.abstraction.RioPWMPort;

/**
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/00
 */
public class PWMLEDArray {
	private final RioPWMPort port;
	public PWMLEDArray(RioPWMPort initPort) {
		port = initPort;
	}

	public void set(double d) {
		System.out.println("new PWMLEDArray(" + port.toString() + ").set(" + d + ") called");
	}
	//TODO
}
