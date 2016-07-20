package org.firstinspires.frc.framework.hardware;

import edu.wpi.first.wpilibj.*;
import org.firstinspires.frc.framework.software.RioCANID;

/**
 * Represents and allows the manipulation of a motor's speed controller, allowing motors to be controlled. Both PWM and CAN interfaces are supported.
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/10
 */
public class MotorController_Individual {
	public static final class CANIsUnsupportedException extends Exception {
		public CANIsUnsupportedException(MotorControllerType m) {
			super(m.name() + " motor controllers do not support CAN");
		}
	}

	private final int rawIndex;
	private MotorControllerType type;
	private final boolean isCAN;

	private Jaguar rawJaguarInstance;
	private SD540 rawSD540Instance;
	private Spark rawSparkInstance;
	private Talon rawTalonInstance;
	private TalonSRX rawTalonSRXInstance;
	private Victor rawVictorInstance;
	private VictorSP rawVictorSPInstance;

	private CANJaguar rawJaguarCANInstance;
	private CANTalon rawTalonSRXCANInstance;

	private boolean isReversed = false;

	public MotorController_Individual(RioHWPort initPort, MotorControllerType initType) {
		if (initPort.getType() != RioHWPort.PortType.PWM) {
			throw new RioHWPort.MismatchedRioPortException(RioHWPort.PortType.PWM, initPort.getType());
		}
		rawIndex = initPort.getIndex();
		type = initType;
		isCAN = false;
		switch (type) {
			case Jaguar:
				rawJaguarInstance = new Jaguar(rawIndex);
				break;
			case SD540:
				rawSD540Instance = new SD540(rawIndex);
				break;
			case Spark:
				rawSparkInstance = new Spark(rawIndex);
				break;
			case Talon:
			case TalonSR:
				rawTalonInstance = new Talon(rawIndex);
				break;
			case TalonSRX:
				rawTalonSRXInstance = new TalonSRX(rawIndex);
				break;
			case Victor:
				rawVictorInstance = new Victor(rawIndex);
				break;
			case VictorSP:
				rawVictorSPInstance = new VictorSP(rawIndex);
				break;
		}
	}
	public MotorController_Individual(RioCANID initPort, MotorControllerType initType) throws CANIsUnsupportedException {
		if (!initType.getIsCANEnabled()) {
			throw new CANIsUnsupportedException(initType);
		}
		rawIndex = initPort.getIDNumber();
		type = initType;
		isCAN = true;
		switch (type) {
			case Jaguar:
				rawJaguarCANInstance = new CANJaguar(rawIndex);
				break;
			case TalonSRX:
				rawTalonSRXCANInstance = new CANTalon(rawIndex);
				break;
		}
	}

	public int getRawIndex() {
		return rawIndex;
	}

	public boolean isCAN() {
		return isCAN;
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
		switch (type) {
			case Jaguar:
				if (isCAN) {
					rawJaguarCANInstance.set(d);
				} else {
					rawJaguarInstance.set(d);
				}
				break;
			case SD540:
				rawSD540Instance.set(d);
				break;
			case Spark:
				rawSparkInstance.set(d);
				break;
			case Talon:
			case TalonSR:
				rawTalonInstance.set(d);
				break;
			case TalonSRX:
				if (isCAN) {
					rawTalonSRXCANInstance.set(d);
				} else {
					rawTalonSRXInstance.set(d);
				}
				break;
			case Victor:
				rawVictorInstance.set(d);
				break;
			case VictorSP:
				rawVictorSPInstance.set(d);
				break;
		}
	}
}
