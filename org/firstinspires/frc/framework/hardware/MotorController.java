package org.firstinspires.frc4739.framework;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Victor;

public class MotorController {
	public static enum MotorControllerType {
		Talon, TalonSR, TalonSRX, Victor
	}
	private RioPWMPort rioPort;
	private MotorControllerType controllerType;
	private Object controllerRaw;

	public MotorController(RioPWMPort port, MotorControllerType type) {
		switch (type) {
			case Talon:
			case TalonSR:
				controllerRaw = new Talon(port.getPortNumber());
				break;
			case TalonSRX:
				controllerRaw = new TalonSRX(port.getPortNumber());
				break;
			case Victor:
				controllerRaw = new Victor(port.getPortNumber());
		}
	}

	public void myMethod() {
		
	}
}
