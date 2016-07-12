package org.firstinspires.frc.framework.hardware;

import org.firstinspires.frc.framework.abstraction.RioCANID;
import org.firstinspires.frc.framework.abstraction.RioPWMPort;
import org.firstinspires.frc.framework.granulation.GenericCANMotorController;
import org.firstinspires.frc.framework.granulation.GenericPWMMotorController;

/**
 * Represents and allows the manipulation of a motor's speed controller, allowing motors to be controlled. Both PWM and CAN interfaces are supported.
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/20
 */
public class MotorController {
	public static final class CANIsUnsupportedException extends Exception {
		public CANIsUnsupportedException(MotorControllerType m) {
			super(m.name() + " motor controllers do not support CAN");
		}
	}

	private final int rawIndex;
	private final MotorControllerType type;
	private final boolean isCAN;
	private GenericPWMMotorController rawPWMInstance;
	private GenericCANMotorController rawCANInstance;
	private boolean isReversed = false;

	public MotorController(RioPWMPort initPort, MotorControllerType initType) {
		rawIndex = initPort.getPortNumber();
		type = initType;
		isCAN = false;
		rawPWMInstance = new GenericPWMMotorController(initPort, type);
	}
	public MotorController(RioCANID initPort, MotorControllerType initType) throws CANIsUnsupportedException {
		if (!initType.getIsCANEnabled()) {
			throw new CANIsUnsupportedException(initType);
		}
		rawIndex = initPort.getIDNumber();
		type = initType;
		isCAN = true;
		rawCANInstance = new GenericCANMotorController(initPort, type);
	}

	public boolean isReversed() {
		return isReversed;
	}

	public void reverse() {
		isReversed = !isReversed;
	}

	public void setSpeed(double d) {
		if (isReversed) {
			d = -d;
		}
		if (isCAN) {
			rawCANInstance.set(d);
		} else {
			rawPWMInstance.set(d);
		}
	}
}
