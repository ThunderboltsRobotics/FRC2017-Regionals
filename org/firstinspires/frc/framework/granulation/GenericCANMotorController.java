package org.firstinspires.frc.framework.granulation;

import edu.wpi.first.wpilibj.CANJaguar.LimitMode;
import edu.wpi.first.wpilibj.CANJaguar.NeutralMode;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDeviceStatus;
import edu.wpi.first.wpilibj.CANTalon.SetValueMotionProfile;
import edu.wpi.first.wpilibj.CANTalon.StatusFrameRate;
import edu.wpi.first.wpilibj.can.CANJNI;
import edu.wpi.first.wpilibj.can.CANMessageNotFoundException;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.hal.CanTalonJNI;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;
import edu.wpi.first.wpilibj.util.AllocationException;
import edu.wpi.first.wpilibj.util.CheckedAllocationException;
import org.firstinspires.frc.framework.abstraction.RioCANID;
import org.firstinspires.frc.framework.abstraction.RobotKiller;
import org.firstinspires.frc.framework.hardware.MotorController;
import org.firstinspires.frc.framework.hardware.MotorController.CANIsUnsupportedException;
import org.firstinspires.frc.framework.hardware.MotorControllerType;
import org.firstinspires.frc.framework.software.WPILibJExemplifiesPoorProgrammingException;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import java.lang.Math;

/**
 * Clutter-free replacement for the CANJaguar and CANTalon (for the TalonSRX model) classes from WPILibJ v2016.0.0.
 * Creating an object to manipulate motor controllers? You probably want MotorController.
 * CANJaguar.java and CANTalon.java (both decompiled) total to < 2600 lines. This file is < 2300 lines.
 * @author FRC 4739 Thunderbolts Robotics
 * @see MotorController
 * @version 2016-07-19/00
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public class GenericCANMotorController implements MotorSafety, PIDOutput, PIDSource, CANSpeedController {
	private static final boolean[] JAGUAR_VERIFIED_REFERENCE = {true, true, true, true, true, true, true, true, true, true, true, true, true, true, true};
	private static final float JAGUAR_MIN_FAULT_TIME = (float) 0.5;
	private static final float JAGUAR_MAX_FAULT_TIME = 3;
	private static final int TALON_MAX_VOLTAGE_FORWARD = 12;
	private static final int TALON_MIN_VOLTAGE_FORWARD = 0;
	private static final int TALON_MAX_VOLTAGE_REVERSE = 0;
	private static final int TALON_MIN_VOLTAGE_REVERSE = -12;
	private static final int TALON_MAX_TRAJECTORY_TIME = 255;
	private static final int TALON_MIN_TRAJECTORY_TIME = 0;

	//Global
	private RioCANID port;
	private MotorControllerType type;
	private CANDeviceControlMode controlMode;
	private ITable iTable;
	private ITableListener iTableListener;

	private boolean isInverted = false;
	private boolean isControlEnabled = true;

	//Jaguar
	private static final int jaguarFirmwareVersionMinimum = 108;
	private static final int jaguarFirmwareVersionMaximum = 3329;
	private enum JaguarVerifiedStatuses {
		ControlMode, LimitMode, NeutralMode,
		SpeedReference, PositionReference,
		ForwardLimit, ReverseLimit,
		P, I, D,
		EncoderCodesPerRevolution, PotentiometerTurns, MaxOutputVoltage, VoltageRampRate, FaultTime
	}
	private boolean[] jaguarReceivedStatusMessages;
	private boolean[] jaguarVerifiedStatuses;
	private byte jaguarHardwareVersion;
	private int jaguarFirmwareVersion;

	//TalonSRX
	private static final double talonGetPIDDelay_s = 0.004;
	private long talonJNIInstanceID;

	//Throwables
	public class AttemptedPIDWithIncorrectControlModeException extends UnsupportedOperationException {
		public AttemptedPIDWithIncorrectControlModeException(String methodName) {
			super("Attempted to call the " + methodName + " method of a CAN " + type.name() + " on " + controlMode.name() + " mode");
		}
	}

	private String implodeStringArray(String separator, String[] strings) {
		String toReturn = strings[0];
		for (int i = 1; i < strings.length; i++) {
			toReturn += separator + strings[i];
		}
		return toReturn;
	}
	private String implodeStringArray(String[] s) {
		return implodeStringArray(", ", s);
	}
	public class ExtraParametersNotApplicableException extends IllegalArgumentException {
		public ExtraParametersNotApplicableException(String methodName, String[] parameters) {
			super(type.name() + "s don't use the extra argument for " + methodName + "(..." + implodeStringArray(parameters) + ")");
		}
	}

	public class InvalidJaguarFirmwareVersionException extends IndexOutOfBoundsException {
		public InvalidJaguarFirmwareVersionException(boolean isAboveMaximum) {
			super("Jaguar #" + m_deviceNumberJaguar + "'s firmware version (" + jaguarFirmwareVersion + ") is " + (isAboveMaximum ? "not FIRST approved (version number above highest approved version)" : "is too old (must be at least version 108 of the FIRST approved firmware)"));
		}
	}

	//from CANJaguar and CANTalon
	private static final Resource allocated = new Resource(63);

	//from CANJaguar and CANTalon
	private double m_value = 0;
	private double m_p = 0;
	private double m_i = 0;
	private double m_d = 0;
	private int m_speedReference = 255;
	private int m_positionReference = 255;

	//from CANJaguar and CANTalon
	private double m_forwardLimit;
	private double m_reverseLimit;
	private double m_maxOutputVoltage;
	private double m_voltageRampRate;
	private double m_busVoltage;
	private double m_outputVoltage;
	private double m_outputCurrent;
	private double m_temperature;
	private double m_position;
	private double m_speed;
	private double m_setPoint;
	private float m_faultTime;
	private short m_encoderCodesPerRev;
	private short m_potentiometerTurns;
	private byte m_limits;
	private int m_profile;
	private int m_codesPerRev;
	private int m_numPotTurns;

	//from CANJaguar and CANTalon
	private FeedbackDevice m_feedbackDevice;
	private LimitMode m_limitMode;
	private MotionProfileStatus motionProfileStatus;
	private MotorSafetyHelper m_safetyHelper;
	private NeutralMode m_neutralMode;
	private PIDSourceType m_pidSource;

	public enum CANDeviceControlMode implements ControlMode {
		PercentVbus(0), Position(1), Speed(2), Current(3), Voltage(4), Follower(5), MotionProfile(6), Disabled(15);

		public final int value;
		CANDeviceControlMode(int value) {
			this.value = value;
		}
		public int getValue() {
			return 0;
		}

		public boolean isPID() {
			switch (this) {
				case Position:
				case Speed:
				case Current:
					return true;
				default:
					return false;
			}
		}

		@SuppressWarnings("unused")
		public boolean isTalonOnly() {
			switch (this) {
				case Follower:
				case MotionProfile:
				case Disabled:
					return true;
				default:
					return false;
			}
		}
	}

	//TODO merge - from CANJaguar and CANTalon
	private byte m_deviceNumberJaguar;
	private int m_deviceNumberTalon;

	private void talonConstructor(int deviceNumber) {
		m_pidSource = PIDSourceType.kDisplacement;
		m_deviceNumberTalon = deviceNumber;
		m_safetyHelper = new MotorSafetyHelper(this);
		m_profile = 0;
		m_setPoint = 0;
		m_codesPerRev = 0;
		m_numPotTurns = 0;
		m_feedbackDevice = FeedbackDevice.QuadEncoder;
		setProfile(m_profile);
		applyControlMode(CANDeviceControlMode.PercentVbus);
		LiveWindow.addActuator("CANTalon", m_deviceNumberTalon, this);
	}

	private void jaguarConstructor(int deviceNumber) {
		m_neutralMode = NeutralMode.Jumper;
		m_encoderCodesPerRev = 0;
		m_potentiometerTurns = 0;
		m_limitMode = LimitMode.SwitchInputsOnly;
		m_forwardLimit = 0;
		m_reverseLimit = 0;
		m_maxOutputVoltage = 12;
		m_voltageRampRate = 0;
		m_faultTime = 0;
		m_busVoltage = 0;
		m_outputVoltage = 0;
		m_outputCurrent = 0;
		m_temperature = 0;
		m_position = 0;
		m_speed = 0;
		m_limits = 0;
		jaguarFirmwareVersion = 0;
		jaguarHardwareVersion = 0;
		jaguarReceivedStatusMessages = new boolean[]{false, false, false};
		jaguarVerifiedStatuses = new boolean[JaguarVerifiedStatuses.values().length];

		try {
			allocated.allocate(deviceNumber - 1);
		} catch (CheckedAllocationException e) {
			throw new AllocationException("GenericCANMotorController device " + e.getMessage() + "(increment index by one)");
		}

		m_deviceNumberJaguar = (byte) deviceNumber;
		controlMode = CANDeviceControlMode.PercentVbus;
		m_safetyHelper = new MotorSafetyHelper(this);
		byte[] data = new byte[8];
		requestMessage(-2147483136);
		requestMessage(520225088);

		for (int e = 0; e < 50; e++) {
			Timer.delay(0.001);
			setupPeriodicStatus();
			updatePeriodicStatus();
			getMessage(512, JaguarCANMessageID.VerificationMask.getValue(), data);
			jaguarFirmwareVersion = unpackINT32(data);
			if (jaguarReceivedStatusMessages[0] && jaguarReceivedStatusMessages[1] && jaguarReceivedStatusMessages[2]) {
				break;
			}
		}

		if (jaguarReceivedStatusMessages[0] && jaguarReceivedStatusMessages[1] && jaguarReceivedStatusMessages[2]) {
			getMessage(520225088, JaguarCANMessageID.VerificationMask.getValue(), data);
			jaguarHardwareVersion = data[0];
			if (jaguarFirmwareVersion < jaguarFirmwareVersionMinimum) {
				throw new InvalidJaguarFirmwareVersionException(false);
			} else if (jaguarFirmwareVersion > jaguarFirmwareVersionMaximum) {
				throw new InvalidJaguarFirmwareVersionException(true);
			}
		} else {
			free();
			throw new CANMessageNotFoundException();
		}
	}

	public GenericCANMotorController(RioCANID p, MotorControllerType m) throws CANMessageNotFoundException, CANIsUnsupportedException {
		port = p;
		type = m;
		switch (m) {
			case Jaguar:
				jaguarConstructor(port.getIDNumber());
				break;
			case TalonSRX:
				talonConstructor(port.getIDNumber());
				talonJNIInstanceID = CanTalonJNI.new_CanTalonSRX(port.getIDNumber());
				break;
			default:
				throw new CANIsUnsupportedException(type);
		}
	}

	@SuppressWarnings("unused")
	public GenericCANMotorController(RioCANID p, MotorControllerType m, int talonSRXCAN_controlPeriod_ms) throws CANMessageNotFoundException, CANIsUnsupportedException {
		port = p;
		type = m;
		switch (m) {
			case Jaguar:
				throw new ExtraParametersNotApplicableException("new GenericCANMotorController", new String[]{"int talonSRXCAN_controlPeriod_ms"});
			case TalonSRX:
				talonConstructor(port.getIDNumber());
				talonJNIInstanceID = CanTalonJNI.new_CanTalonSRX(p.getIDNumber(), talonSRXCAN_controlPeriod_ms);
				break;
			default:
				throw new CANIsUnsupportedException(type);
		}
	}

	@SuppressWarnings("unused")
	public GenericCANMotorController(RioCANID p, MotorControllerType m, int talonSRXCAN_controlPeriod_ms, int talonSRXCANEnablePeriod_ms) throws CANMessageNotFoundException, CANIsUnsupportedException {
		port = p;
		type = m;
		switch (m) {
			case Jaguar:
				throw new ExtraParametersNotApplicableException("new GenericCANMotorController", new String[]{"int talonSRXCAN_controlPeriod_ms", "int talonSRXCANEnablePeriod_ms"});
			case TalonSRX:
				talonConstructor(port.getIDNumber());
				talonJNIInstanceID = CanTalonJNI.new_CanTalonSRX(p.getIDNumber(), talonSRXCAN_controlPeriod_ms, talonSRXCANEnablePeriod_ms);
				break;
			default:
				throw new CANIsUnsupportedException(type);
		}
	}

	/**
	 * Gets the last speed target that was set to this device.
	 * @return Speed target
	 */
	public double get() {
		switch (type) {
			case Jaguar:
				return m_value;
			case TalonSRX:
				switch (controlMode) {
					case Current:
						return getOutputVoltage();
					case Voltage:
						return ScaleNativeUnitsToRpm(m_feedbackDevice, (long) CanTalonJNI.GetSensorVelocity(talonJNIInstanceID));
					case Follower:
						return ScaleNativeUnitsToRotations(m_feedbackDevice, CanTalonJNI.GetSensorPosition(talonJNIInstanceID));
					case MotionProfile:
						return getOutputCurrent();
					default:
						return CanTalonJNI.GetAppliedThrottle(talonJNIInstanceID) / 1023;
				}
			default:
				return RobotKiller.doubleKill();
		}
	}

	public void set(double outputValue, byte syncGroup) {
		switch (type) {
			case Jaguar:
				byte[] data = new byte[8];
				if (isControlEnabled) {
					JaguarCANMessageID messageID;
					byte dataSize;
					switch (controlMode) {
						case PercentVbus:
							messageID = JaguarCANMessageID.set_Percent;
							dataSize = packPercentage(data, isInverted ? -outputValue : outputValue);
							break;
						case Current:
							messageID = JaguarCANMessageID.set_Current;
							dataSize = packFXP16_16(data, isInverted ? -outputValue : outputValue);
							break;
						case Speed:
							messageID = JaguarCANMessageID.set_Speed;
							dataSize = packFXP16_16(data, outputValue);
							break;
						case Position:
							messageID = JaguarCANMessageID.set_Position;
							dataSize = packFXP8_8(data, outputValue);
							break;
						case Voltage:
							messageID = JaguarCANMessageID.set_Voltage;
							dataSize = packFXP8_8(data, isInverted ? -outputValue : outputValue);
							break;
						default:
							return;
					}
					if (syncGroup != 0) {
						data[dataSize++] = syncGroup;
					}
					sendMessage(messageID.getValue(), data, dataSize, 20);
					if (m_safetyHelper != null) {
						m_safetyHelper.feed();
					}
				}
				m_value = outputValue;
				if (jaguarVerifiedStatuses != JAGUAR_VERIFIED_REFERENCE) {
					verify();
				}
				break;
			case TalonSRX:
				throw new ExtraParametersNotApplicableException("set", new String[]{"byte syncGroup"});
		}
	}

	/**
	 * Jaguar: Calls set(d, 0).
	 * TalonSRX: Sets the target speed based on the parameter d and the current control mode, reversing it if set to reverse.
	 */
	public void set(double d) {
		switch (type) {
			case Jaguar:
				set(d, (byte) 0);
				break;
			case TalonSRX:
				m_safetyHelper.feed();
				if (isControlEnabled) {
					m_setPoint = d;
					switch (controlMode) {
						case PercentVbus:
							CanTalonJNI.Set(talonJNIInstanceID, isInverted ? -d : d);
							break;
						case Position:
							CanTalonJNI.SetDemand(talonJNIInstanceID, (int) d);
							break;
						case Speed:
							CanTalonJNI.SetDemand(talonJNIInstanceID, 256 * (int) (isInverted ? -d : d));
							break;
						case Current:
							CanTalonJNI.SetDemand(talonJNIInstanceID, ScaleVelocityToNativeUnits(m_feedbackDevice, isInverted ? -d : d));
							break;
						case Voltage:
							CanTalonJNI.SetDemand(talonJNIInstanceID, ScaleRotationsToNativeUnits(m_feedbackDevice, d));
							break;
						case Follower:
							CanTalonJNI.SetDemand(talonJNIInstanceID, 1000 * (int) (isInverted ? -d : d));
							break;
						case MotionProfile:
							CanTalonJNI.SetDemand(talonJNIInstanceID, (int) d);
							break;
					}
					CanTalonJNI.SetModeSelect(talonJNIInstanceID, controlMode.value);
				}
				break;
		}
	}

	/**
	 * Calls getPosition() if in a PID control mode, throws an exception otherwise.
	 * For use as an input device for PID (implements PIDSource).
	 * @return Position (as calculated by the motor controller)
	 */
	public double pidGet() {
		if (controlMode.isPID()) {
			return getPosition();
		} else {
			throw new AttemptedPIDWithIncorrectControlModeException("pidWrite");
		}
	}

	/**
	 * Calls set(d) if in a PID control mode, throws an exception otherwise.
	 * For use as an output device for PID (implements PIDOutput).
	 * @param d Speed target
	 */
	public void pidWrite(double d) {
		if (controlMode.isPID()) {
			set(d);
		} else {
			throw new AttemptedPIDWithIncorrectControlModeException("pidWrite");
		}
	}

	public boolean isEnabled() {
		return isControlEnabled;
	}

	public boolean getInverted() {
		return isInverted;
	}

	public void setInverted(boolean isInverted) {
		this.isInverted = isInverted;
	}

	public double getP() {
		switch (type) {
			case Jaguar:
				switch (controlMode) {
					case Current:
					case Speed:
					case Position:
						return m_p;
					case PercentVbus:
					case Voltage:
					default:
						throw new AttemptedPIDWithIncorrectControlModeException("setD");
				}
			case TalonSRX:
				CanTalonJNI.RequestParam(talonJNIInstanceID, m_profile == 0 ? CanTalonJNI.param_t.eProfileParamSlot0_P.value : CanTalonJNI.param_t.eProfileParamSlot1_P.value);
				Timer.delay(talonGetPIDDelay_s);
				return CanTalonJNI.GetPgain(talonJNIInstanceID, m_profile);
			default:
				return RobotKiller.doubleKill();
		}
	}

	public void setP(double p) {
		switch (type) {
			case Jaguar:
				JaguarCANMessageID messageID;
				switch(controlMode) {
					case Current:
						messageID = JaguarCANMessageID.setP_Current;
						break;
					case Speed:
						messageID = JaguarCANMessageID.setP_Speed;
						break;
					case Position:
						messageID = JaguarCANMessageID.setP_Position;
						break;
					default:
						throw new AttemptedPIDWithIncorrectControlModeException("setD");
				}
				byte[] data = new byte[8];
				byte dataSize = packFXP16_16(data, p);
				sendMessage(messageID.getValue(), data, dataSize);
				m_p = p;
				jaguarVerifiedStatuses[JaguarVerifiedStatuses.P.ordinal()] = false;
				break;
			case TalonSRX:
				CanTalonJNI.SetPgain(talonJNIInstanceID, m_profile, p);
				break;
		}
	}

	public double getI() {
		switch (type) {
			case Jaguar:
				switch (controlMode) {
					case Current:
					case Speed:
					case Position:
						return m_i;
					default:
						throw new AttemptedPIDWithIncorrectControlModeException("setD");
				}
			case TalonSRX:
				CanTalonJNI.RequestParam(talonJNIInstanceID, m_profile == 0 ? CanTalonJNI.param_t.eProfileParamSlot0_I.value : CanTalonJNI.param_t.eProfileParamSlot1_I.value);
				Timer.delay(talonGetPIDDelay_s);
				return CanTalonJNI.GetIgain(talonJNIInstanceID, m_profile);
			default:
				return RobotKiller.doubleKill();
		}
	}

	public void setI(double i) {
		switch (type) {
			case Jaguar:
				JaguarCANMessageID messageID;
				switch(controlMode) {
					case Current:
						messageID = JaguarCANMessageID.setI_Current;
						break;
					case Speed:
						messageID = JaguarCANMessageID.setI_Speed;
						break;
					case Position:
						messageID = JaguarCANMessageID.setI_Position;
						break;
					default:
						throw new AttemptedPIDWithIncorrectControlModeException("setD");
				}
				byte[] data = new byte[8];
				byte dataSize = packFXP16_16(data, i);
				sendMessage(messageID.getValue(), data, dataSize);
				m_i = i;
				jaguarVerifiedStatuses[JaguarVerifiedStatuses.I.ordinal()] = false;
				break;
			case TalonSRX:
				CanTalonJNI.SetIgain(talonJNIInstanceID, m_profile, i);
				break;
		}
	}

	public double getD() {
		switch (type) {
			case Jaguar:
				switch (controlMode) {
					case Current:
					case Speed:
					case Position:
						return m_d;
					case PercentVbus:
					case Voltage:
					default:
						throw new AttemptedPIDWithIncorrectControlModeException("setD");
				}
			case TalonSRX:
				CanTalonJNI.RequestParam(talonJNIInstanceID, m_profile == 0 ? CanTalonJNI.param_t.eProfileParamSlot0_D.value : CanTalonJNI.param_t.eProfileParamSlot1_D.value);
				Timer.delay(talonGetPIDDelay_s);
				return CanTalonJNI.GetDgain(talonJNIInstanceID, m_profile);
			default:
				return RobotKiller.doubleKill();
		}
	}

	public void setD(double d) {
		switch (type) {
			case Jaguar:
				JaguarCANMessageID messageID;
				switch(controlMode) {
					case Current:
						messageID = JaguarCANMessageID.setD_Current;
						break;
					case Speed:
						messageID = JaguarCANMessageID.setD_Speed;
						break;
					case Position:
						messageID = JaguarCANMessageID.setD_Position;
						break;
					default:
						throw new AttemptedPIDWithIncorrectControlModeException("setD");
				}
				byte[] data = new byte[8];
				byte dataSize = packFXP16_16(data, d);
				sendMessage(messageID.getValue(), data, dataSize);
				m_d = d;
				jaguarVerifiedStatuses[JaguarVerifiedStatuses.D.ordinal()] = false;
				break;
			case TalonSRX:
				CanTalonJNI.SetDgain(talonJNIInstanceID, m_profile, d);
				break;
		}
	}

	public double getF() {
		if (m_profile == 0) {
			CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot0_F.value);
		} else {
			CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot1_F.value);
		}
		Timer.delay(talonGetPIDDelay_s);
		return CanTalonJNI.GetFgain(talonJNIInstanceID, m_profile);
	}

	public void setF(double f) {
		CanTalonJNI.SetFgain(talonJNIInstanceID, m_profile, f);
	}

	@SuppressWarnings("unused")
	public double getIZone() {
		if (m_profile == 0) {
			CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot0_IZone.value);
		} else {
			CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot1_IZone.value);
		}
		Timer.delay(talonGetPIDDelay_s);
		return (double) CanTalonJNI.GetIzone(talonJNIInstanceID, m_profile);
	}

	public void setIZone(int iZone) {
		CanTalonJNI.SetIzone(talonJNIInstanceID, m_profile, iZone);
	}

	@SuppressWarnings("unused")
	public double getCloseLoopRampRate() {
		if (m_profile == 0) {
			CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot0_CloseLoopRampRate.value);
		} else {
			CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot1_CloseLoopRampRate.value);
		}
		Timer.delay(talonGetPIDDelay_s);
		return CanTalonJNI.GetCloseLoopRampRate(talonJNIInstanceID, m_profile) * 12000 / 1023;
	}

	public void setCloseLoopRampRate(double rampRate) {
		CanTalonJNI.SetCloseLoopRampRate(talonJNIInstanceID, m_profile, (int) (rampRate * 1023 / 12000));
	}

	public void setPID(double p, double i, double d) {
		switch (type) {
			case Jaguar:
				setP(p);
				setI(i);
				setD(d);
				break;
			case TalonSRX:
				setPID(p, i, d, 0, 0, 0, m_profile);
				break;
		}
	}

	public void setPID(double p, double i, double d, double f, int iZone, double closeLoopRampRate, int profile) {
		switch (type) {
			case Jaguar:
				throw new ExtraParametersNotApplicableException("setPID", new String[]{"double f", "int iZone", "double closeLoopRampRate", "int profile"});
			case TalonSRX:
				if (profile != 0 && profile != 1) {
					throw new IllegalArgumentException("Talon PID profile must be 0 or 1.");
				} else {
					m_profile = profile;
					setProfile(m_profile);
					setP(p);
					setI(i);
					setD(d);
					setF(f);
					setIZone(iZone);
					setCloseLoopRampRate(closeLoopRampRate);
				}
				break;
		}
	}

//TODO end manually organized code
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO
//TODO start automatically organized code

	static void sendMessageHelper(int messageID, byte[] data, int dataSize, int period) throws CANMessageNotFoundException {
		JaguarCANMessageID[] trustedMessageIDs = {
				JaguarCANMessageID.set_Percent, JaguarCANMessageID.set_Current, JaguarCANMessageID.set_Speed, JaguarCANMessageID.set_Position, JaguarCANMessageID.set_Voltage,
				JaguarCANMessageID.enableControl_Percent, JaguarCANMessageID.enableControl_Current, JaguarCANMessageID.enableControl_Speed, JaguarCANMessageID.enableControl_Position, JaguarCANMessageID.enableControl_Voltage
		};
		ByteBuffer byteBuffer;
		for (JaguarCANMessageID id : trustedMessageIDs) {
			if ((JaguarCANMessageID.sendMessageHelper_tooMuchData.getValue() & messageID) == id.getValue()) {
				if (dataSize > 6) {
					throw new RuntimeException("CAN message has too much data.");
				}
				byteBuffer = ByteBuffer.allocateDirect(dataSize + 2);
				//TODO IRL see if these changes work
				//byteBuffer.put(0, (byte) 0);
				//byteBuffer.put(1, (byte) 0);
				byteBuffer.put((byte) 0);
				byteBuffer.put((byte) 0);
				for (byte b = 0; b < dataSize; b++) {
					//byteBuffer.put(b + 2, data[b]);
					byteBuffer.put(data[b]);
				}
				CANJNI.FRCNetworkCommunicationCANSessionMuxSendMessage(messageID, byteBuffer, period);
				return;
			}
		}
		if (data != null) {
			byteBuffer = ByteBuffer.allocateDirect(dataSize);
			for (byte b = 0; b < dataSize; b++) {
				byteBuffer.put(b, data[b]);
			}
		} else {
			byteBuffer = null;
		}
		CANJNI.FRCNetworkCommunicationCANSessionMuxSendMessage(messageID, byteBuffer, period);
	}

	private static void swap16(int x, byte[] buffer) {
		for (int i = 0; i < 2; i++) {
			buffer[i] = (byte) (x >> (i * 8) & 255);
		}
	}
	private static void swap32(int x, byte[] buffer) {
		for (int i = 0; i < 4; i++) {
			buffer[i] = (byte) (x >> (i * 8) & 255);
		}
	}

	private static byte packPercentage(byte[] buffer, double value) {
		if (value < -1) {
			value = -1;
		}
		if (value > 1) {
			value = 1;
		}
		swap16((int) (value * 32767), buffer);
		return (byte) 2;
	}

	private static byte packFXP8_8(byte[] buffer, double value) {
		swap16((int) (value * 256), buffer);
		return (byte) 2;
	}

	private static byte packFXP16_16(byte[] buffer, double value) {
		swap32((int) (value * 65536), buffer);
		return (byte) 4;
	}

	private static byte packINT16(byte[] buffer, short value) {
		swap16(value, buffer);
		return (byte) 2;
	}

	private static short unpack16(byte[] buffer, int offset) {
		return (short)(buffer[offset] & 255 | (short)(buffer[offset + 1] << 8) & '\uff00');
	}

	private static int unpack32(byte[] buffer, int offset) {
		return buffer[offset] & 255 | buffer[offset + 1] << 8 & '\uff00' | buffer[offset + 2] << 16 & 16711680 | buffer[offset + 3] << 24 & -16777216;
	}

	private static double unpackPercentage(byte[] buffer) {
		return (double) unpack16(buffer, 0) / 32767;
	}

	private static double unpackFXP8_8(byte[] buffer) {
		return (double) unpack16(buffer, 0) / 256;
	}

	private static double unpackFXP16_16(byte[] buffer) {
		return (double) unpack32(buffer, 0) / 65536;
	}

	private static short unpackINT16(byte[] buffer) {
		return unpack16(buffer, 0);
	}

	private static int unpackINT32(byte[] buffer) {
		return unpack32(buffer, 0);
	}

	private void free() {
		allocated.free(m_deviceNumberJaguar - 1);
		m_safetyHelper = null;
		JaguarCANMessageID messageID;
		switch(controlMode) {
			case PercentVbus:
				messageID = JaguarCANMessageID.set_Percent;
				break;
			case Current:
				messageID = JaguarCANMessageID.set_Current;
				break;
			case Speed:
				messageID = JaguarCANMessageID.set_Speed;
				break;
			case Position:
				messageID = JaguarCANMessageID.set_Position;
				break;
			case Voltage:
				messageID = JaguarCANMessageID.set_Voltage;
				break;
			default:
				return;
		}
		CANJNI.FRCNetworkCommunicationCANSessionMuxSendMessage(m_deviceNumberJaguar | messageID.getValue(), null, -1);
		configMaxOutputVoltage(12);
	}

	public double getSetpoint() {
		switch (type) {
			case Jaguar:
				return get();
			case TalonSRX:
				return m_setPoint;
			default:
				return RobotKiller.doubleKill();
		}
	}

	public void setSetpoint(double d) {
		set(d);
	}

	public double getError() {
		switch (type) {
			case Jaguar:
				return get() - getPosition();
			case TalonSRX:
				return (double) getClosedLoopError();
			default:
				return RobotKiller.doubleKill();
		}
	}

	public void reset() {
		switch (type) {
			case Jaguar:
				set(m_value);
				break;
			case TalonSRX:
				CanTalonJNI.SetParam(talonJNIInstanceID, CanTalonJNI.param_t.ePidIaccum.value, 0);
				break;
		}
		disableControl();
	}

	public void verify() throws CANMessageNotFoundException {
		byte[] data = new byte[8];
		try {
			getMessage(JaguarCANMessageID.verify_Init.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
			if (data[0] != 0) {
				data[0] = 1;
				sendMessage(JaguarCANMessageID.verify_Init.getValue(), data, 1);
				jaguarVerifiedStatuses = new boolean[]{
						false, false, false,
						false, false,
						false, false,
						false, false, false,
						false, false, false, false, false
				};
				jaguarReceivedStatusMessages[0] = false;
				jaguarReceivedStatusMessages[1] = false;
				jaguarReceivedStatusMessages[2] = false;
				int[] messageIDs = new int[]{
						JaguarCANMessageID.setSpeedReference.getValue(), JaguarCANMessageID.setPositionReference.getValue(),
						JaguarCANMessageID.setP_Current.getValue(), JaguarCANMessageID.setP_Speed.getValue(), JaguarCANMessageID.setP_Position.getValue(),
						JaguarCANMessageID.setI_Current.getValue(), JaguarCANMessageID.setI_Speed.getValue(), JaguarCANMessageID.setI_Position.getValue(),
						JaguarCANMessageID.setD_Current.getValue(), JaguarCANMessageID.setD_Speed.getValue(), JaguarCANMessageID.setD_Position.getValue(),
						JaguarCANMessageID.configEncoderCodesPerRev.getValue(), JaguarCANMessageID.configPotentiometerTurns.getValue(), JaguarCANMessageID.configNeutralMode.getValue(), JaguarCANMessageID.configLimitMode.getValue(), JaguarCANMessageID.configReverseLimit.getValue(), JaguarCANMessageID.configMaxOutputVoltage.getValue(),
						JaguarCANMessageID.setVoltageRampRate_Percent.getValue(), JaguarCANMessageID.setVoltageRampRate_Voltage.getValue(),
						33693184, JaguarCANMessageID.configForwardLimit.getValue()
				};
				for (int message : messageIDs) {
					getMessage(message, JaguarCANMessageID.VerificationMask.getValue(), data);
				}
			}
		} catch (CANMessageNotFoundException e) {
			requestMessage(JaguarCANMessageID.verify_Init.getValue());
		}

		if (isControlEnabled && !jaguarVerifiedStatuses[JaguarVerifiedStatuses.ControlMode.ordinal()]) {
			try {
				getMessage(33691200, JaguarCANMessageID.VerificationMask.getValue(), data);
				if (controlMode == CANDeviceControlMode.values()[data[0]]) {
					jaguarVerifiedStatuses[JaguarVerifiedStatuses.ControlMode.ordinal()] = true;
				} else {
					enableControl();
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33691200);
			}
		}

		byte var28;
		if (!jaguarVerifiedStatuses[JaguarVerifiedStatuses.SpeedReference.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.setSpeedReference.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				var28 = data[0];
				if (m_speedReference == var28) {
					jaguarVerifiedStatuses[JaguarVerifiedStatuses.SpeedReference.ordinal()] = true;
				} else {
					setSpeedReference(m_speedReference);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.setSpeedReference.getValue());
			}
		}
		if (!jaguarVerifiedStatuses[JaguarVerifiedStatuses.PositionReference.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.setPositionReference.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				var28 = data[0];
				if (m_positionReference == var28) {
					jaguarVerifiedStatuses[JaguarVerifiedStatuses.PositionReference.ordinal()] = true;
				} else {
					setPositionReference(m_positionReference);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.setPositionReference.getValue());
			}
		}

		int var29;
		double var30;
		if (!jaguarVerifiedStatuses[JaguarVerifiedStatuses.P.ordinal()]) {
			switch(controlMode) {
				case Current:
					var29 = JaguarCANMessageID.setP_Current.getValue();
					break;
				case Speed:
					var29 = JaguarCANMessageID.setP_Speed.getValue();
					break;
				case Position:
					var29 = JaguarCANMessageID.setP_Position.getValue();
					break;
				default:
					var29 = 0;
			}
			try {
				getMessage(var29, JaguarCANMessageID.VerificationMask.getValue(), data);
				var30 = unpackFXP16_16(data);
				if (FXP16_EQ(m_p, var30)) {
					jaguarVerifiedStatuses[JaguarVerifiedStatuses.P.ordinal()] = true;
				} else {
					setP(m_p);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(var29);
			}
		}
		if (!jaguarVerifiedStatuses[JaguarVerifiedStatuses.I.ordinal()]) {
			switch(controlMode) {
				case Current:
					var29 = JaguarCANMessageID.setI_Current.getValue();
					break;
				case Speed:
					var29 = JaguarCANMessageID.setI_Speed.getValue();
					break;
				case Position:
					var29 = JaguarCANMessageID.setI_Position.getValue();
					break;
				default:
					var29 = 0;
			}
			try {
				getMessage(var29, JaguarCANMessageID.VerificationMask.getValue(), data);
				var30 = unpackFXP16_16(data);
				if (FXP16_EQ(m_i, var30)) {
					jaguarVerifiedStatuses[JaguarVerifiedStatuses.I.ordinal()] = true;
				} else {
					setI(m_i);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(var29);
			}
		}
		if (!jaguarVerifiedStatuses[JaguarVerifiedStatuses.D.ordinal()]) {
			switch(controlMode) {
				case Current:
					var29 = JaguarCANMessageID.setD_Current.getValue();
					break;
				case Speed:
					var29 = JaguarCANMessageID.setD_Speed.getValue();
					break;
				case Position:
					var29 = JaguarCANMessageID.setD_Position.getValue();
					break;
				default:
					var29 = 0;
			}
			try {
				getMessage(var29, JaguarCANMessageID.VerificationMask.getValue(), data);
				var30 = unpackFXP16_16(data);
				if (FXP16_EQ(m_d, var30)) {
					jaguarVerifiedStatuses[JaguarVerifiedStatuses.D.ordinal()] = true;
				} else {
					setD(m_d);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(var29);
			}
		}

		if (!jaguarVerifiedStatuses[JaguarVerifiedStatuses.NeutralMode.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.configNeutralMode.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				if (NeutralMode.valueOf(data[0]) == m_neutralMode) {
					jaguarVerifiedStatuses[JaguarVerifiedStatuses.NeutralMode.ordinal()] = true;
				} else {
					configNeutralMode(m_neutralMode);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.configNeutralMode.getValue());
			}
		}

		short var32;
		if (!jaguarVerifiedStatuses[JaguarVerifiedStatuses.EncoderCodesPerRevolution.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.configEncoderCodesPerRev.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				var32 = unpackINT16(data);
				if (var32 == m_encoderCodesPerRev) {
					jaguarVerifiedStatuses[JaguarVerifiedStatuses.EncoderCodesPerRevolution.ordinal()] = true;
				} else {
					configEncoderCodesPerRev(m_encoderCodesPerRev);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.configEncoderCodesPerRev.getValue());
			}
		}
		if (!jaguarVerifiedStatuses[JaguarVerifiedStatuses.PotentiometerTurns.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.configPotentiometerTurns.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				var32 = unpackINT16(data);
				if (var32 == m_potentiometerTurns) {
					jaguarVerifiedStatuses[JaguarVerifiedStatuses.PotentiometerTurns.ordinal()] = true;
				} else {
					configPotentiometerTurns(m_potentiometerTurns);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.configPotentiometerTurns.getValue());
			}
		}

		if (!jaguarVerifiedStatuses[JaguarVerifiedStatuses.LimitMode.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.configLimitMode.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				LimitMode var33 = LimitMode.valueOf(data[0]);
				if (var33 == m_limitMode) {
					jaguarVerifiedStatuses[JaguarVerifiedStatuses.LimitMode.ordinal()] = true;
				} else {
					configLimitMode(m_limitMode);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.configLimitMode.getValue());
			}
		}

		double var34;
		if (!jaguarVerifiedStatuses[JaguarVerifiedStatuses.ForwardLimit.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.configForwardLimit.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				var34 = unpackFXP16_16(data);
				if (FXP16_EQ(var34, m_forwardLimit)) {
					jaguarVerifiedStatuses[JaguarVerifiedStatuses.ForwardLimit.ordinal()] = true;
				} else {
					configForwardLimit(m_forwardLimit);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.configForwardLimit.getValue());
			}
		}
		if (!jaguarVerifiedStatuses[JaguarVerifiedStatuses.ReverseLimit.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.configReverseLimit.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				var34 = unpackFXP16_16(data);
				if (FXP16_EQ(var34, m_reverseLimit)) {
					jaguarVerifiedStatuses[JaguarVerifiedStatuses.ReverseLimit.ordinal()] = true;
				} else {
					configReverseLimit(m_reverseLimit);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.configReverseLimit.getValue());
			}
		}
		if (!jaguarVerifiedStatuses[JaguarVerifiedStatuses.MaxOutputVoltage.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.configMaxOutputVoltage.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				var34 = unpackFXP8_8(data);
				if (Math.abs(var34 - m_maxOutputVoltage) < 0.1) {
					jaguarVerifiedStatuses[JaguarVerifiedStatuses.MaxOutputVoltage.ordinal()] = true;
				} else {
					configMaxOutputVoltage(m_maxOutputVoltage);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.configMaxOutputVoltage.getValue());
			}
		}
		if (!jaguarVerifiedStatuses[JaguarVerifiedStatuses.VoltageRampRate.ordinal()]) {
			if (controlMode == CANDeviceControlMode.PercentVbus) {
				try {
					getMessage(JaguarCANMessageID.setVoltageRampRate_Percent.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
					var34 = unpackPercentage(data);
					if (FXP16_EQ(var34, m_voltageRampRate)) {
						jaguarVerifiedStatuses[JaguarVerifiedStatuses.VoltageRampRate.ordinal()] = true;
					} else {
						setVoltageRampRate(m_voltageRampRate);
					}
				} catch (CANMessageNotFoundException e) {
					requestMessage(JaguarCANMessageID.setVoltageRampRate_Percent.getValue());
				}
			}
		} else if (controlMode == CANDeviceControlMode.Voltage) {
			try {
				getMessage(JaguarCANMessageID.setVoltageRampRate_Voltage.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				var34 = unpackFXP8_8(data);
				if (FXP8_EQ(var34, m_voltageRampRate)) {
					jaguarVerifiedStatuses[JaguarVerifiedStatuses.VoltageRampRate.ordinal()] = true;
				} else {
					setVoltageRampRate(m_voltageRampRate);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.setVoltageRampRate_Voltage.getValue());
			}
		}

		if (!jaguarVerifiedStatuses[JaguarVerifiedStatuses.FaultTime.ordinal()]) {
			try {
				getMessage(33693184, JaguarCANMessageID.VerificationMask.getValue(), data);
				var32 = unpackINT16(data);
				if ((int)((double) m_faultTime * 1000) == var32) {
					jaguarVerifiedStatuses[JaguarVerifiedStatuses.FaultTime.ordinal()] = true;
				} else {
					configFaultTime(m_faultTime);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33693184);
			}
		}

		if (!jaguarReceivedStatusMessages[0] || !jaguarReceivedStatusMessages[1] || !jaguarReceivedStatusMessages[2]) {
			setupPeriodicStatus();
			getTemperature();
			getPosition();
			updatePeriodicStatus();
		}
	}

	/**
	 * Renamed -> disableControl()
	 * @deprecated
	 */
	@Deprecated
	public void disable() {}

	/**
	 * Renamed -> enableControl()
	 * @deprecated
	 */
	@Deprecated
	public void enable() {}

	public void disableControl() {
		switch (type) {
			case Jaguar:
				JaguarCANMessageID[] disableControlMessageIDs = {
						JaguarCANMessageID.disableControl_Percent, JaguarCANMessageID.disableControl_Current, JaguarCANMessageID.disableControl_Speed, JaguarCANMessageID.disableControl_Position, JaguarCANMessageID.disableControl_Voltage
				};
				for (JaguarCANMessageID id : disableControlMessageIDs) {
					sendMessage(id.getValue(), new byte[0], 0);
				}
				JaguarCANMessageID[] setMessageIDs = {
						JaguarCANMessageID.set_Percent, JaguarCANMessageID.set_Current, JaguarCANMessageID.set_Speed, JaguarCANMessageID.set_Position, JaguarCANMessageID.set_Voltage
				};
				for (JaguarCANMessageID id : setMessageIDs) {
					sendMessage(id.getValue(), new byte[0], 0, -1);
				}
				break;
			case TalonSRX:
				CanTalonJNI.SetModeSelect(talonJNIInstanceID, CANDeviceControlMode.Disabled.value);
				break;
		}
		isControlEnabled = false;
	}

	public void enableControl() {
		switch (type) {
			case Jaguar:
				JaguarCANMessageID messageID = JaguarCANMessageID.NULL;
				switch(controlMode) {
					case PercentVbus:
						messageID = JaguarCANMessageID.enableControl_Percent;
						break;
					case Current:
						messageID = JaguarCANMessageID.enableControl_Current;
						break;
					case Speed:
						messageID = JaguarCANMessageID.enableControl_Speed;
						break;
					case Position:
						messageID = JaguarCANMessageID.enableControl_Position;
						break;
					case Voltage:
						messageID = JaguarCANMessageID.enableControl_Voltage;
						break;
				}
				sendMessage(messageID.getValue(), controlMode == CANDeviceControlMode.Speed ? new byte[8] : new byte[0], controlMode == CANDeviceControlMode.Speed ? packFXP16_16(new byte[8], 0) : 0);
				break;
			case TalonSRX:
				setControlMode(controlMode);
				break;
		}
		isControlEnabled = true;
	}

	private void setSpeedReference(int reference) {
		sendMessage(JaguarCANMessageID.setSpeedReference.getValue(), new byte[]{(byte) reference}, 1);
		m_speedReference = reference;
		jaguarVerifiedStatuses[JaguarVerifiedStatuses.SpeedReference.ordinal()] = false;
	}

	private void setPositionReference(int reference) {
		sendMessage(JaguarCANMessageID.setPositionReference.getValue(), new byte[]{(byte) reference}, 1);
		m_positionReference = reference;
		jaguarVerifiedStatuses[JaguarVerifiedStatuses.PositionReference.ordinal()] = false;
	}

	public void configEncoderCodesPerRev_Talon(int codesPerRev) {
		m_codesPerRev = codesPerRev;
		setParameter(CanTalonJNI.param_t.eNumberEncoderCPR, (double) m_codesPerRev);
	}

	public void setPercentMode() {
		setControlMode(CANDeviceControlMode.PercentVbus);
		setPositionReference(255);
		setSpeedReference(255);
	}

	public void setPercentMode_Encoder(int codesPerRev, boolean isQuadEncoder) {
		setControlMode(CANDeviceControlMode.PercentVbus);
		configEncoderCodesPerRev(codesPerRev);
		if (isQuadEncoder) {
			setPositionReference(0);
			setSpeedReference(3);
		} else {
			setPositionReference(255);
			setSpeedReference(0);
		}
	}

	public void setPercentMode_Potentiometer() {
		setControlMode(CANDeviceControlMode.PercentVbus);
		setPositionReference(1);
		setSpeedReference(255);
		configPotentiometerTurns(1);
	}

	public void setCurrentMode(double p, double i, double d) {
		setControlMode(CANDeviceControlMode.Current);
		setPositionReference(255);
		setSpeedReference(255);
		setPID(p, i, d);
	}

	public void setCurrentMode_Encoder(double p, double i, double d, int codesPerRev, boolean isQuadEncoder) {
		setControlMode(CANDeviceControlMode.Current);
		configEncoderCodesPerRev(codesPerRev);
		setPID(p, i, d);
		if (isQuadEncoder) {
			setPositionReference(0);
			setSpeedReference(3);
		} else {
			setPositionReference(255);
			setSpeedReference(255);
		}
	}

	public void setCurrentMode_Potentiometer(double p, double i, double d) {
		setControlMode(CANDeviceControlMode.Current);
		setPositionReference(1);
		setSpeedReference(255);
		configPotentiometerTurns(1);
		setPID(p, i, d);
	}

	public void setSpeedMode(double p, double i, double d, int codesPerRev, boolean isQuadEncoder) {
		setControlMode(CANDeviceControlMode.Speed);
		configEncoderCodesPerRev(codesPerRev);
		setPID(p, i, d);
		if (isQuadEncoder) {
			setPositionReference(0);
			setSpeedReference(3);
		} else {
			setPositionReference(255);
			setSpeedReference(0);
		}
	}

	public void setPositionMode_QuadEncoder(double p, double i, double d, int codesPerRev) {
		setControlMode(CANDeviceControlMode.Position);
		setPositionReference(0);
		configEncoderCodesPerRev(codesPerRev);
		setPID(p, i, d);
	}

	public void setPositionMode_Potentiometer(double p, double i, double d) {
		setControlMode(CANDeviceControlMode.Position);
		setPositionReference(1);
		configPotentiometerTurns(1);
		setPID(p, i, d);
	}

	public void setVoltageMode() {
		setControlMode(CANDeviceControlMode.Voltage);
		setPositionReference(255);
		setSpeedReference(255);
	}

	public void setVoltageMode_Encoder(int codesPerRev, boolean isQuadEncoder) {
		setControlMode(CANDeviceControlMode.Voltage);
		configEncoderCodesPerRev(codesPerRev);
		if (isQuadEncoder) {
			setPositionReference(0);
			setSpeedReference(3);
		} else {
			setPositionReference(255);
			setSpeedReference(0);
		}
	}

	public void setVoltageMode_Potentiometer() {
		setControlMode(CANDeviceControlMode.Voltage);
		setPositionReference(1);
		setSpeedReference(255);
		configPotentiometerTurns(1);
	}

	public ControlMode getControlMode() {
		return controlMode;
	}

	public void setControlMode(int i) {
		try {
			throw new WPILibJExemplifiesPoorProgrammingException("Why would you have setControlMode(int i), only to take the ith-constructed enum value?");
		} catch (WPILibJExemplifiesPoorProgrammingException e) {
			e.printStackTrace();
		}
	}
	public void setControlMode(CANDeviceControlMode m) {
		switch (type) {
			case Jaguar:
				disableControl();
				controlMode = m;
				jaguarVerifiedStatuses[JaguarVerifiedStatuses.ControlMode.ordinal()] = false;
				break;
			case TalonSRX:
				if (controlMode != m) {
					applyControlMode(m);
				}
				break;
		}
	}

	public double getBusVoltage() {
		switch (type) {
			case Jaguar:
				updatePeriodicStatus();
				return m_busVoltage;
			case TalonSRX:
				return CanTalonJNI.GetBatteryV(talonJNIInstanceID);
			default:
				return RobotKiller.doubleKill();
		}
	}

	public double getOutputVoltage() {
		switch (type) {
			case Jaguar:
				updatePeriodicStatus();
				return m_outputVoltage;
			case TalonSRX:
				return getBusVoltage() * (double) CanTalonJNI.GetAppliedThrottle(talonJNIInstanceID) / 1023;
			default:
				return RobotKiller.doubleKill();
		}
	}

	public double getOutputCurrent() {
		switch (type) {
			case Jaguar:
				updatePeriodicStatus();
				return m_outputCurrent;
			case TalonSRX:
				return CanTalonJNI.GetCurrent(talonJNIInstanceID);
			default:
				return RobotKiller.doubleKill();
		}
	}

	public double getTemperature() {
		switch (type) {
			case Jaguar:
				updatePeriodicStatus();
				return m_temperature;
			case TalonSRX:
				return CanTalonJNI.GetTemp(talonJNIInstanceID);
			default:
				return RobotKiller.doubleKill();
		}
	}

	public double getPosition() {
		switch (type) {
			case Jaguar:
				updatePeriodicStatus();
				return m_position;
			case TalonSRX:
				return ScaleNativeUnitsToRotations(m_feedbackDevice, CanTalonJNI.GetSensorPosition(talonJNIInstanceID));
			default:
				return RobotKiller.doubleKill();
		}
	}

	public void setPosition(double pos) {
		int nativePos = ScaleRotationsToNativeUnits(m_feedbackDevice, pos);
		CanTalonJNI.SetSensorPosition(talonJNIInstanceID, nativePos);
	}

	public double getSpeed() {
		switch (type) {
			case Jaguar:
				updatePeriodicStatus();
				return m_speed;
			case TalonSRX:
				return ScaleNativeUnitsToRpm(m_feedbackDevice, (long) CanTalonJNI.GetSensorVelocity(talonJNIInstanceID));
			default:
				return RobotKiller.doubleKill();
		}
	}

	public boolean getForwardLimitOK() {
		updatePeriodicStatus();
		return (m_limits & 1) != 0;
	}

	public boolean getReverseLimitOK() {
		updatePeriodicStatus();
		return (m_limits & 2) != 0;
	}

	public void setVoltageRampRate(double rampRate) {
		switch (type) {
			case Jaguar:
				byte[] data = new byte[8];
				byte dataSize;
				JaguarCANMessageID message;
				switch(controlMode) {
					case PercentVbus:
						dataSize = packPercentage(data, rampRate / (m_maxOutputVoltage * 1000));
						message = JaguarCANMessageID.setVoltageRampRate_Percent;
						break;
					case Voltage:
						dataSize = packFXP8_8(data, rampRate / 1000);
						message = JaguarCANMessageID.setVoltageRampRate_Voltage;
						break;
					default:
						throw new AttemptedPIDWithIncorrectControlModeException("setVoltageRampRate");
				}
				sendMessage(message.getValue(), data, dataSize);
				break;
			case TalonSRX:
				CanTalonJNI.SetRampThrottle(talonJNIInstanceID, (int) (rampRate * 1023 / 1200));
				break;
		}
	}

	public int getFirmwareVersion() {
		switch (type) {
			case Jaguar:
				return jaguarFirmwareVersion;
			case TalonSRX:
				CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eFirmVers.value);
				Timer.delay(talonGetPIDDelay_s);
				return CanTalonJNI.GetParamResponseInt32(talonJNIInstanceID, CanTalonJNI.param_t.eFirmVers.value);
			default:
				return RobotKiller.intKill();
		}
	}

	public byte getHardwareVersion() {
		return jaguarHardwareVersion;
	}

	private void configNeutralMode(NeutralMode mode) {
		sendMessage(JaguarCANMessageID.configNeutralMode.getValue(), new byte[]{mode.value}, 1);
		m_neutralMode = mode;
		jaguarVerifiedStatuses[JaguarVerifiedStatuses.NeutralMode.ordinal()] = false;
	}

	private void configEncoderCodesPerRev(int codesPerRev) {
		byte[] data = new byte[8];
		byte dataSize = packINT16(data, (short) codesPerRev);
		sendMessage(JaguarCANMessageID.configEncoderCodesPerRev.getValue(), data, dataSize);
		m_encoderCodesPerRev = (short) codesPerRev;
		jaguarVerifiedStatuses[JaguarVerifiedStatuses.EncoderCodesPerRevolution.ordinal()] = false;
	}

	private void configPotentiometerTurns(int turns) {
		switch (type) {
			case Jaguar:
				byte[] data = new byte[8];
				byte dataSize = packINT16(data, (short) turns);
				sendMessage(JaguarCANMessageID.configPotentiometerTurns.getValue(), data, dataSize);
				m_potentiometerTurns = (short) turns;
				jaguarVerifiedStatuses[JaguarVerifiedStatuses.PotentiometerTurns.ordinal()] = false;
				break;
			case TalonSRX:
				m_numPotTurns = turns;
				setParameter(CanTalonJNI.param_t.eNumberPotTurns, (double) m_numPotTurns);
				break;
		}
	}

	private void configSoftPositionLimits(double forwardLimitPosition, double reverseLimitPosition) {
		configLimitMode(LimitMode.SoftPositionLimits);
		configForwardLimit(forwardLimitPosition);
		configReverseLimit(reverseLimitPosition);
	}

	private void disableSoftPositionLimits() {
		configLimitMode(LimitMode.SwitchInputsOnly);
	}

	private void configLimitMode(LimitMode mode) {
		sendMessage(JaguarCANMessageID.configLimitMode.getValue(), new byte[]{mode.value}, 1);
	}

	private void configForwardLimit(double forwardLimitPosition) {
		byte[] data = new byte[8];
		byte dataSize = packFXP16_16(data, forwardLimitPosition);
		int dataSize1 = dataSize + 1;
		data[dataSize] = 1;
		sendMessage(JaguarCANMessageID.configForwardLimit.getValue(), data, dataSize1);
		m_forwardLimit = forwardLimitPosition;
		jaguarVerifiedStatuses[JaguarVerifiedStatuses.ForwardLimit.ordinal()] = false;
	}

	private void configReverseLimit(double reverseLimitPosition) {
		byte[] data = new byte[8];
		byte dataSize = packFXP16_16(data, reverseLimitPosition);
		int dataSize1 = dataSize + 1;
		data[dataSize] = 1;
		sendMessage(JaguarCANMessageID.configReverseLimit.getValue(), data, dataSize1);
		m_reverseLimit = reverseLimitPosition;
		jaguarVerifiedStatuses[JaguarVerifiedStatuses.ReverseLimit.ordinal()] = false;
	}

	private void configMaxOutputVoltage(double voltage) {
		byte[] data = new byte[8];
		byte dataSize = packFXP8_8(data, voltage);
		sendMessage(JaguarCANMessageID.configMaxOutputVoltage.getValue(), data, dataSize);
		m_maxOutputVoltage = voltage;
		jaguarVerifiedStatuses[JaguarVerifiedStatuses.MaxOutputVoltage.ordinal()] = false;
	}

	private void configFaultTime(float faultTime) {
		byte[] data = new byte[8];
		faultTime = Math.min(Math.max(faultTime, JAGUAR_MIN_FAULT_TIME), JAGUAR_MAX_FAULT_TIME);
		byte dataSize = packINT16(data, (short)((int)((double) faultTime * 1000)));
		sendMessage(33693184, data, dataSize);
		m_faultTime = faultTime;
		jaguarVerifiedStatuses[JaguarVerifiedStatuses.FaultTime.ordinal()] = false;
	}

	private void sendMessage(int messageID, byte[] data, int dataSize, int period) {
		sendMessageHelper(messageID | m_deviceNumberJaguar, data, dataSize, period);
	}
	private void sendMessage(int messageID, byte[] data, int dataSize) {
		sendMessage(messageID, data, dataSize, 0);
	}

	private void requestMessage(int messageID, int period) {
		sendMessageHelper(messageID | m_deviceNumberJaguar, null, 0, period);
	}
	private void requestMessage(int messageID) {
		requestMessage(messageID, 0);
	}

	private void getMessage(int messageID, int messageMask, byte[] data) throws CANMessageNotFoundException {
		ByteBuffer targetedMessageID = ByteBuffer.allocateDirect(4);
		targetedMessageID.order(ByteOrder.LITTLE_ENDIAN);
		targetedMessageID.asIntBuffer().put(0, (messageID | m_deviceNumberJaguar) & JaguarCANMessageID.VerificationMask.getValue());
		ByteBuffer timeStamp = ByteBuffer.allocateDirect(4);
		ByteBuffer dataBuffer = CANJNI.FRCNetworkCommunicationCANSessionMuxReceiveMessage(targetedMessageID.asIntBuffer(), messageMask, timeStamp);
		if (data != null) {
			for (int i = 0; i < dataBuffer.capacity(); i++) {
				data[i] = dataBuffer.get(i);
			}
		}
	}

	private void setupPeriodicStatus() {
		byte[] data = new byte[8];
		byte dataSize = packINT16(data, (short) 20);
		sendMessage(33691648, data, dataSize);
		sendMessage(33691712, data, dataSize);
		sendMessage(33691776, data, dataSize);

		byte[] kMessage0Data = new byte[]{(byte) 3, (byte) 4, (byte) 1, (byte) 2, (byte) 5, (byte) 6, (byte) 7, (byte) 8};
		byte[] kMessage1Data = new byte[]{(byte) 9, (byte) 10, (byte) 11, (byte) 12, (byte) 13, (byte) 14, (byte) 15, (byte) 16};
		byte[] kMessage2Data = new byte[]{(byte) 18, (byte) 19, (byte) 0, (byte) 0, (byte) 0, (byte) 0, (byte) 0, (byte) 0};
		dataSize = 8;
		sendMessage(33691904, kMessage0Data, dataSize);
		sendMessage(33691968, kMessage1Data, dataSize);
		sendMessage(33692032, kMessage2Data, dataSize);
	}

	private void updatePeriodicStatus() throws CANMessageNotFoundException {
		byte[] data = new byte[8];

		getMessage(33692160, JaguarCANMessageID.VerificationMask.getValue(), data);
		m_busVoltage = unpackFXP8_8(new byte[]{data[0], data[1]});
		m_outputVoltage = unpackPercentage(new byte[]{data[2], data[3]}) * m_busVoltage;
		m_outputCurrent = unpackFXP8_8(new byte[]{data[4], data[5]});
		m_temperature = unpackFXP8_8(new byte[]{data[6], data[7]});
		jaguarReceivedStatusMessages[0] = true;

		getMessage(33692224, JaguarCANMessageID.VerificationMask.getValue(), data);
		m_position = unpackFXP16_16(new byte[]{data[0], data[1], data[2], data[3]});
		m_speed = unpackFXP16_16(new byte[]{data[4], data[5], data[6], data[7]});
		jaguarReceivedStatusMessages[1] = true;

		getMessage(33692288, JaguarCANMessageID.VerificationMask.getValue(), data);
		m_limits = data[0];
		jaguarReceivedStatusMessages[2] = true;
	}

	private boolean FXP8_EQ(double a, double b) {
		return Math.floor(a * 256) == Math.floor(b * 256);
	}

	private boolean FXP16_EQ(double a, double b) {
		return Math.floor(a * 65536) == Math.floor(b * 65536);
	}

	public double getExpiration() {
		return m_safetyHelper.getExpiration();
	}

	public void setExpiration(double timeout) {
		m_safetyHelper.setExpiration(timeout);
	}

	public boolean isAlive() {
		return m_safetyHelper.isAlive();
	}

	public boolean isSafetyEnabled() {
		return m_safetyHelper.isSafetyEnabled();
	}

	public void setSafetyEnabled(boolean enabled) {
		m_safetyHelper.setSafetyEnabled(enabled);
	}

	public String getDescription() {
		return "GenericCANMotorController ID " + m_deviceNumberJaguar;
	}

	public int getDeviceID() {
		return m_deviceNumberJaguar;
	}

	/** @deprecated */
	@Deprecated
	public void stopMotor() {
		disableControl();
	}

	public void initTable(ITable table) {
		iTable = table;
		updateTable();
	}

	public ITable getTable() {
		return iTable;
	}

	public void startLiveWindowMode() {
		set(0);
		iTableListener = createTableListener();
		iTable.addTableListener(iTableListener, true);
	}

	public void stopLiveWindowMode() {
		iTable.removeTableListener(iTableListener);
	}

	public PIDSourceType getPIDSourceType() {
		return m_pidSource;
	}

	public void setPIDSourceType(PIDSourceType pidSource) {
		m_pidSource = pidSource;
	}

	public void delete() {
		disableControl();
		if (talonJNIInstanceID != 0) {
			CanTalonJNI.delete_CanTalonSRX(talonJNIInstanceID);
			talonJNIInstanceID = 0;
		}
	}

	public void reverseSensor(boolean flip) {
		CanTalonJNI.SetRevFeedbackSensor(talonJNIInstanceID, flip ? 1 : 0);
	}

	public void reverseOutput(boolean flip) {
		CanTalonJNI.SetRevMotDuringCloseLoopEn(talonJNIInstanceID, flip ? 1 : 0);
	}

	public int getEncPosition() {
		return CanTalonJNI.GetEncPosition(talonJNIInstanceID);
	}

	public void setEncPosition(int newPosition) {
		setParameter(CanTalonJNI.param_t.eEncPosition, (double) newPosition);
	}

	public int getEncVelocity() {
		return CanTalonJNI.GetEncVel(talonJNIInstanceID);
	}

	public int getPulseWidthPosition() {
		return CanTalonJNI.GetPulseWidthPosition(talonJNIInstanceID);
	}

	public void setPulseWidthPosition(int newPosition) {
		setParameter(CanTalonJNI.param_t.ePwdPosition, (double) newPosition);
	}

	public int getPulseWidthVelocity() {
		return CanTalonJNI.GetPulseWidthVelocity(talonJNIInstanceID);
	}

	public int getPulseWidthRiseToFallUs() {
		return CanTalonJNI.GetPulseWidthRiseToFallUs(talonJNIInstanceID);
	}

	public int getPulseWidthRiseToRiseUs() {
		return CanTalonJNI.GetPulseWidthRiseToRiseUs(talonJNIInstanceID);
	}

	public FeedbackDeviceStatus isSensorPresent(FeedbackDevice feedbackDevice) {
		FeedbackDeviceStatus toReturn = FeedbackDeviceStatus.FeedbackStatusUnknown;
		switch(feedbackDevice.ordinal()) {
			case 6:
			case 7:
			case 8:
				if (CanTalonJNI.IsPulseWidthSensorPresent(talonJNIInstanceID) == 0) {
					toReturn = FeedbackDeviceStatus.FeedbackStatusNotPresent;
				} else {
					toReturn = FeedbackDeviceStatus.FeedbackStatusPresent;
				}
			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
			default:
				return toReturn;
		}
	}

	public int getNumberOfQuadIdxRises() {
		return CanTalonJNI.GetEncIndexRiseEvents(talonJNIInstanceID);
	}

	public int getPinStateQuadA() {
		return CanTalonJNI.GetQuadApin(talonJNIInstanceID);
	}

	public int getPinStateQuadB() {
		return CanTalonJNI.GetQuadBpin(talonJNIInstanceID);
	}

	public int getPinStateQuadIdx() {
		return CanTalonJNI.GetQuadIdxpin(talonJNIInstanceID);
	}

	public void setAnalogPosition(int newPosition) {
		setParameter(CanTalonJNI.param_t.eAinPosition, (double) newPosition);
	}

	public int getAnalogInPosition() {
		return CanTalonJNI.GetAnalogInWithOv(talonJNIInstanceID);
	}

	public int getAnalogInRaw() {
		return getAnalogInPosition() & 1023;
	}

	public int getAnalogInVelocity() {
		return CanTalonJNI.GetAnalogInVel(talonJNIInstanceID);
	}

	public int getClosedLoopError() {
		return CanTalonJNI.GetCloseLoopErr(talonJNIInstanceID);
	}

	public void setAllowableClosedLoopErr(int allowableCloseLoopError) {
		if (m_profile == 0) {
			setParameter(CanTalonJNI.param_t.eProfileParamSlot0_AllowableClosedLoopErr, (double) allowableCloseLoopError);
		} else {
			setParameter(CanTalonJNI.param_t.eProfileParamSlot1_AllowableClosedLoopErr, (double) allowableCloseLoopError);
		}
	}

	public boolean isFwdLimitSwitchClosed() {
		return CanTalonJNI.GetLimitSwitchClosedFor(talonJNIInstanceID) == 0;
	}

	public boolean isRevLimitSwitchClosed() {
		return CanTalonJNI.GetLimitSwitchClosedRev(talonJNIInstanceID) == 0;
	}

	public boolean getBrakeEnableDuringNeutral() {
		return CanTalonJNI.GetBrakeIsEnabled(talonJNIInstanceID) != 0;
	}

	private void applyControlMode(CANDeviceControlMode m) {
		controlMode = m;
		if (m == CANDeviceControlMode.Disabled) {
			isControlEnabled = false;
		}

		CanTalonJNI.SetModeSelect(talonJNIInstanceID, CANDeviceControlMode.Disabled.value);
		UsageReporting.report(52, m_deviceNumberTalon + 1, m.value);
	}

	public void setFeedbackDevice(FeedbackDevice device) {
		m_feedbackDevice = device;
		CanTalonJNI.SetFeedbackDeviceSelect(talonJNIInstanceID, device.value);
	}

	public void setStatusFrameRateMs(StatusFrameRate stateFrame, int periodMs) {
		CanTalonJNI.SetStatusFrameRate(talonJNIInstanceID, stateFrame.value, periodMs);
	}

	public long getIAccum() {
		CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.ePidIaccum.value);
		Timer.delay(talonGetPIDDelay_s);
		return (long) CanTalonJNI.GetParamResponseInt32(talonJNIInstanceID, CanTalonJNI.param_t.ePidIaccum.value);
	}

	public void setVoltageCompensationRampRate(double rampRate) {
		CanTalonJNI.SetVoltageCompensationRate(talonJNIInstanceID, rampRate / 1000);
	}

	public void setProfile(int profile) {
		if (profile != 0 && profile != 1) {
			throw new IllegalArgumentException("Talon PID profile must be 0 or 1.");
		} else {
			m_profile = profile;
			CanTalonJNI.SetProfileSlotSelect(talonJNIInstanceID, m_profile);
		}
	}

	/** @deprecated */
	@Deprecated
	public void stopMotor_Talon() {
		disableControl();
	}

	public int getForwardSoftLimit() {
		return CanTalonJNI.GetForwardSoftLimit(talonJNIInstanceID);
	}

	public void setForwardSoftLimit(double forwardLimit) {
		int nativeLimitPos = ScaleRotationsToNativeUnits(m_feedbackDevice, forwardLimit);
		CanTalonJNI.SetForwardSoftLimit(talonJNIInstanceID, nativeLimitPos);
	}

	public void enableForwardSoftLimit(boolean enable) {
		CanTalonJNI.SetForwardSoftEnable(talonJNIInstanceID, enable ? 1 : 0);
	}

	public boolean isForwardSoftLimitEnabled() {
		return CanTalonJNI.GetForwardSoftEnable(talonJNIInstanceID) != 0;
	}

	public int getReverseSoftLimit() {
		return CanTalonJNI.GetReverseSoftLimit(talonJNIInstanceID);
	}

	public void setReverseSoftLimit(double reverseLimit) {
		int nativeLimitPos = ScaleRotationsToNativeUnits(m_feedbackDevice, reverseLimit);
		CanTalonJNI.SetReverseSoftLimit(talonJNIInstanceID, nativeLimitPos);
	}

	private void enableReverseSoftLimit(boolean enable) {
		CanTalonJNI.SetReverseSoftEnable(talonJNIInstanceID, enable ? 1 : 0);
	}

	private boolean isReverseSoftLimitEnabled() {
		return CanTalonJNI.GetReverseSoftEnable(talonJNIInstanceID) != 0;
	}

	private void configMaxOutputVoltage_Talon(double voltage) {
		configPeakOutputVoltage(voltage, -voltage);
	}

	private void configPeakOutputVoltage(double forwardVoltage, double reverseVoltage) {
		setParameter(CanTalonJNI.param_t.ePeakPosOutput, Math.min(Math.max(forwardVoltage, TALON_MIN_VOLTAGE_FORWARD), TALON_MAX_VOLTAGE_FORWARD) * 1023 / 12);
		setParameter(CanTalonJNI.param_t.ePeakNegOutput, Math.min(Math.max(reverseVoltage, TALON_MIN_VOLTAGE_REVERSE), TALON_MAX_VOLTAGE_REVERSE) * 1023 / 12);
	}

	private void configNominalOutputVoltage(double forwardVoltage, double reverseVoltage) {
		setParameter(CanTalonJNI.param_t.eNominalPosOutput, Math.min(Math.max(forwardVoltage, TALON_MIN_VOLTAGE_FORWARD), TALON_MAX_VOLTAGE_FORWARD) * 1023 / 12);
		setParameter(CanTalonJNI.param_t.eNominalNegOutput, Math.min(Math.max(reverseVoltage, TALON_MIN_VOLTAGE_REVERSE), TALON_MAX_VOLTAGE_REVERSE) * 1023 / 12);
	}

	public void setParameter(CanTalonJNI.param_t paramEnum, double value) {
		CanTalonJNI.SetParam(talonJNIInstanceID, paramEnum.value, value);
	}

	public double getParameter(CanTalonJNI.param_t paramEnum) {
		CanTalonJNI.RequestParam(talonJNIInstanceID, paramEnum.value);
		Timer.delay(talonGetPIDDelay_s);
		return CanTalonJNI.GetParamResponse(talonJNIInstanceID, paramEnum.value);
	}

	public void clearStickyFaults() {
		CanTalonJNI.ClearStickyFaults(talonJNIInstanceID);
	}

	public void enableLimitSwitch(boolean forward, boolean reverse) {
		int mask = 4 + (forward ? 2 : 0) + (reverse ? 1 : 0);
		CanTalonJNI.SetOverrideLimitSwitchEn(talonJNIInstanceID, mask);
	}

	public void ConfigFwdLimitSwitchNormallyOpen(boolean normallyOpen) {
		CanTalonJNI.SetParam(talonJNIInstanceID, CanTalonJNI.param_t.eOnBoot_LimitSwitch_Forward_NormallyClosed.value, normallyOpen ? 0 : 1);
	}

	public void ConfigRevLimitSwitchNormallyOpen(boolean normallyOpen) {
		CanTalonJNI.SetParam(talonJNIInstanceID, CanTalonJNI.param_t.eOnBoot_LimitSwitch_Reverse_NormallyClosed.value, normallyOpen ? 0 : 1);
	}

	public void enableBrakeMode(boolean brake) {
		CanTalonJNI.SetOverrideBrakeType(talonJNIInstanceID, brake ? 2 : 1);
	}

	public int getFaultOverTemp() {
		return CanTalonJNI.GetFault_OverTemp(talonJNIInstanceID);
	}

	public int getFaultUnderVoltage() {
		return CanTalonJNI.GetFault_UnderVoltage(talonJNIInstanceID);
	}

	public int getFaultForLim() {
		return CanTalonJNI.GetFault_ForLim(talonJNIInstanceID);
	}

	public int getFaultRevLim() {
		return CanTalonJNI.GetFault_RevLim(talonJNIInstanceID);
	}

	public int getFaultHardwareFailure() {
		return CanTalonJNI.GetFault_HardwareFailure(talonJNIInstanceID);
	}

	public int getFaultForSoftLim() {
		return CanTalonJNI.GetFault_ForSoftLim(talonJNIInstanceID);
	}

	public int getFaultRevSoftLim() {
		return CanTalonJNI.GetFault_RevSoftLim(talonJNIInstanceID);
	}

	public int getStickyFaultOverTemp() {
		return CanTalonJNI.GetStckyFault_OverTemp(talonJNIInstanceID);
	}

	public int getStickyFaultUnderVoltage() {
		return CanTalonJNI.GetStckyFault_UnderVoltage(talonJNIInstanceID);
	}

	public int getStickyFaultForLim() {
		return CanTalonJNI.GetStckyFault_ForLim(talonJNIInstanceID);
	}

	public int getStickyFaultRevLim() {
		return CanTalonJNI.GetStckyFault_RevLim(talonJNIInstanceID);
	}

	public int getStickyFaultForSoftLim() {
		return CanTalonJNI.GetStckyFault_ForSoftLim(talonJNIInstanceID);
	}

	public int getStickyFaultRevSoftLim() {
		return CanTalonJNI.GetStckyFault_RevSoftLim(talonJNIInstanceID);
	}

	private double GetNativeUnitsPerRotationScalar(FeedbackDevice devToLookup) {
		double toReturn = 0;
		boolean scalingAvail = false;
		switch(devToLookup.ordinal()) {
			case 1:
				switch(m_feedbackDevice.ordinal()) {
					case 1:
					case 2:
					case 3:
					case 4:
					case 5:
					case 6:
					default:
						break;
					case 7:
					case 8:
						toReturn = 4096;
						scalingAvail = true;
				}

				if (!scalingAvail && 0 != m_codesPerRev) {
					toReturn = (double)(4 * m_codesPerRev);
					scalingAvail = true;
				}
				break;
			case 2:
			case 3:
				if (0 != m_numPotTurns) {
					toReturn = 1024 / (double) m_numPotTurns;
					scalingAvail = true;
				}
				break;
			case 4:
			case 5:
				if (0 != m_codesPerRev) {
					toReturn = (double)(m_codesPerRev);
					scalingAvail = true;
				}
				break;
			case 6:
			case 7:
			case 8:
				toReturn = 4096;
				scalingAvail = true;
		}

		return scalingAvail ? toReturn : 0;
	}

	private int ScaleRotationsToNativeUnits(FeedbackDevice devToLookup, double fullRotations) {
		double scalar = GetNativeUnitsPerRotationScalar(devToLookup);
		return (int) (fullRotations * (scalar > 0 ? scalar : 1));
	}

	private int ScaleVelocityToNativeUnits(FeedbackDevice devToLookup, double rpm) {
		double scalar = GetNativeUnitsPerRotationScalar(devToLookup);
		return (int) (rpm * (scalar > 0 ? scalar / 600 : 1));
	}

	private double ScaleNativeUnitsToRotations(FeedbackDevice devToLookup, int nativePosition) {
		double scalar = GetNativeUnitsPerRotationScalar(devToLookup);
		return nativePosition / (scalar > 0 ? scalar : 1);
	}

	private double ScaleNativeUnitsToRpm(FeedbackDevice devToLookup, long nativeVelocity) {
		double scalar = GetNativeUnitsPerRotationScalar(devToLookup);
		return nativeVelocity / (scalar > 0 ? scalar / 600 : 1);
	}

	public void enableZeroSensorPositionOnIndex(boolean enable, boolean risingEdge) {
		setParameter(CanTalonJNI.param_t.eQuadIdxPolarity, risingEdge ? 1 : 0);
		setParameter(CanTalonJNI.param_t.eClearPositionOnIdx, enable ? 1 : 0);
	}

	public void changeMotionControlFramePeriod(int periodMs) {
		CanTalonJNI.ChangeMotionControlFramePeriod(talonJNIInstanceID, periodMs);
	}

	public void clearMotionProfileTrajectories() {
		CanTalonJNI.ClearMotionProfileTrajectories(talonJNIInstanceID);
	}

	public int getMotionProfileTopLevelBufferCount() {
		return CanTalonJNI.GetMotionProfileTopLevelBufferCount(talonJNIInstanceID);
	}

	public boolean pushMotionProfileTrajectory(TrajectoryPoint trajectoryPoint) {
		if (isMotionProfileTopLevelBufferFull()) {
			return false;
		}
		int targetPos = ScaleRotationsToNativeUnits(m_feedbackDevice, trajectoryPoint.position);
		int targetVel = ScaleVelocityToNativeUnits(m_feedbackDevice, trajectoryPoint.velocity);
		int profileSlotSelect = trajectoryPoint.profileSlotSelect > 0 ? 1 : 0;
		int timeDurMs = Math.min(Math.max(trajectoryPoint.timeDurMs, TALON_MIN_TRAJECTORY_TIME), TALON_MAX_TRAJECTORY_TIME);
		CanTalonJNI.PushMotionProfileTrajectory(talonJNIInstanceID, targetPos, targetVel, profileSlotSelect, timeDurMs, trajectoryPoint.velocityOnly ? 1 : 0, trajectoryPoint.isLastPoint ? 1 : 0, trajectoryPoint.zeroPos ? 1 : 0);
		return true;
	}

	public boolean isMotionProfileTopLevelBufferFull() {
		return CanTalonJNI.IsMotionProfileTopLevelBufferFull(talonJNIInstanceID);
	}

	public void processMotionProfileBuffer() {
		CanTalonJNI.ProcessMotionProfileBuffer(talonJNIInstanceID);
	}

	public void getMotionProfileStatus() {
		CanTalonJNI.GetMotionProfileStatus(talonJNIInstanceID, this, motionProfileStatus);
	}

	public void setMotionProfileStatusFromJNI(int flags, int profileSlotSelect, int targPos, int targVel, int topBufferRem, int topBufferCnt, int btmBufferCnt, int outputEnable) {
		motionProfileStatus = new MotionProfileStatus(topBufferRem, topBufferCnt, btmBufferCnt, (flags & 2) > 0, (flags & 4) > 0, (flags & 1) > 0, SetValueMotionProfile.valueOf(outputEnable));
		motionProfileStatus.activePoint.isLastPoint = (flags & 8) > 0;
		motionProfileStatus.activePoint.velocityOnly = (flags & 16) > 0;
		motionProfileStatus.activePoint.position = ScaleNativeUnitsToRotations(m_feedbackDevice, targPos);
		motionProfileStatus.activePoint.velocity = ScaleNativeUnitsToRpm(m_feedbackDevice, (long) targVel);
		motionProfileStatus.activePoint.profileSlotSelect = profileSlotSelect;
		motionProfileStatus.activePoint.zeroPos = false;
		motionProfileStatus.activePoint.timeDurMs = 0;
	}

	public void clearMotionProfileHasUnderrun() {
		setParameter(CanTalonJNI.param_t.eMotionProfileHasUnderrunErr, 0);
	}

	private static class MotionProfileStatus {
		final int topBufferRem;
		final int topBufferCnt;
		final int btmBufferCnt;
		final boolean hasUnderrun;
		final boolean isUnderrun;
		final boolean activePointValid;
		final TrajectoryPoint activePoint = new TrajectoryPoint();
		final SetValueMotionProfile outputEnable;

		MotionProfileStatus(int topBufferRem, int topBufferCnt, int btmBufferCnt, boolean hasUnderrun, boolean isUnderrun, boolean activePointValid, SetValueMotionProfile outputEnable) {
			this.topBufferRem = topBufferRem;
			this.topBufferCnt = topBufferCnt;
			this.btmBufferCnt = btmBufferCnt;
			this.hasUnderrun = hasUnderrun;
			this.isUnderrun = isUnderrun;
			this.activePointValid = activePointValid;
			this.outputEnable = outputEnable;
		}
	}

	private static class TrajectoryPoint {
		double position;
		double velocity;
		int timeDurMs;
		int profileSlotSelect;
		boolean velocityOnly;
		boolean isLastPoint;
		boolean zeroPos;
	}

	/*
	switch (type) {
		case Jaguar:
			break;
		case TalonSRX:
			break;
		default:
			return RobotKiller.typeKill();
	}
	*/
}
