package org.firstinspires.frc.framework.hardware;

import org.firstinspires.frc.framework.software.RioCANID;
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

	private final MotorControllerType type;
	private final boolean isCAN;
	private GenericPWMMotorController rawPWMCopyInstance;
	private GenericCANMotorController rawCANInstance;
	private boolean isReversed = false;

	@SuppressWarnings("SameParameterValue")
	public MotorController(RioCANID initPort, MotorControllerType initType) throws CANIsUnsupportedException {
		if (initType.getIsCANEnabled()) {
			throw new CANIsUnsupportedException(initType);
		}
		type = initType;
		isCAN = true;
		rawCANInstance = new GenericCANMotorController(initPort, type);
	}
	public MotorController(RioHWPort port, MotorControllerType type) {
		switch (port.getType()) {
			case PWM:
				this.type = type;
				isCAN = false;
				rawPWMCopyInstance = GenericPWMMotorController.builder(port, type);
				break;
			default:
				throw new RioHWPort.MismatchedRioPortException(RioHWPort.PortType.PWM, port.getType());
		}
	}

	@SuppressWarnings("BooleanMethodIsAlwaysInverted")
	public boolean isReversed() {
		return isReversed;
	}

	public void reverse() {
		isReversed = !isReversed;
	}

	public void setSpeed(double d) {
		if (isReversed) d = -d;
		if (isCAN) {
			rawCANInstance.set(d);
		} else {
			rawPWMCopyInstance.set(d);
		}
	}
}
