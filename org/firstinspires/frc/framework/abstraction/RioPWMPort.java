package org.firstinspires.frc4739.framework;

public class RioPWMPort {
	private int port;

	public RioPWMPort(int port) {
		if (port < 0 || 9 < port) {
			throw new ArrayIndexOutOfBoundsException("PWM ports on the Rio go from 0 to 9, " + port + " given");
		}
		this.port = port;
	}

	public int getPortNumber() {
		return port;
	}
}
