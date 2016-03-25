package org.firstinspires.frc.framework.hardware;

import org.firstinspires.frc.framework.abstraction.*;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Victor;

public class MotorController {
	public enum MotorControllerType {
		Talon, TalonSR, TalonSRX, Victor
	}
	private RioPWMPort port;
	private MotorControllerType type;
	private Object rawControllerInstance;

	public MotorController(RioPWMPort initPort, MotorControllerType initType) {
		port = initPort;
		type = initType;
		switch (type) {
			case Talon:
			case TalonSR:
				rawControllerInstance = new Talon(port.getPortNumber());
				break;
			case TalonSRX:
				rawControllerInstance = new TalonSRX(port.getPortNumber());
				break;
			case Victor:
				rawControllerInstance = new Victor(port.getPortNumber());
		}
	}
}
