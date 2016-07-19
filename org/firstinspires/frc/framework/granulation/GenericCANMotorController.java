package org.firstinspires.frc.framework.granulation;

import edu.wpi.first.wpilibj.CANJaguar.LimitMode;
import edu.wpi.first.wpilibj.CANJaguar.NeutralMode;
import edu.wpi.first.wpilibj.CANSpeedController;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDeviceStatus;
import edu.wpi.first.wpilibj.CANTalon.SetValueMotionProfile;
import edu.wpi.first.wpilibj.CANTalon.StatusFrameRate;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Resource;
import edu.wpi.first.wpilibj.Timer;
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

import static org.firstinspires.frc.framework.General.implodeStringArray;

/**
 * Clutter-free replacement for the CANJaguar and CANTalon (for the TalonSRX model) classes from WPILibJ v2016.0.0.
 * Creating an object to manipulate motor controllers? You probably want MotorController.
 * CANJaguar.java and CANTalon.java (both decompiled) total to < 2600 lines. This file is < 2300 lines.
 * @author FRC 4739 Thunderbolts Robotics
 * @see MotorController
 * @version 2016-07-19/01
 */
public class GenericCANMotorController implements CANSpeedController, MotorSafety, PIDSource {
	//Global
	private final RioCANID port;
	private final MotorControllerType type;

	private CANDeviceControlMode controlMode;
	private ITable iTable;
	private ITableListener iTableListener;
	private boolean isInverted = false;
	private boolean isControlEnabled = true;

	//Jaguar
	private static final boolean[] JAGUAR_VERIFIED_REFERENCE = {true, true, true, true, true, true, true, true, true, true, true, true, true, true, true};
	private static final float JAGUAR_FAULT_TIME_MIN = (float) 0.5;
	private static final float JAGUAR_FAULT_TIME_MAX = 3;
	private static final int JAGUAR_FIRMWARE_VERSION_MIN = 108;
	private static final int JAGUAR_FIRMWARE_VERSION_MAX = 3329;
	private enum JaguarVerifiedStatuses {
		ControlMode, LimitMode, NeutralMode,
		SpeedReference, PositionReference,
		ForwardLimit, ReverseLimit,
		P, I, D,
		EncoderCodesPerRevolution, PotentiometerTurns, MaxOutputVoltage, VoltageRampRate, FaultTime
	}
	private boolean[] jagReceivedStatusMessages;
	private boolean[] jagVerifiedStatuses;
	private byte jagHardwareVersion;
	private int jagFirmwareVersion;

	//TalonSRX
	private static final double TALON_GET_PID_DELAY_S = 0.004;
	private static final int TALON_TRAJECTORY_TIME_MAX = 255;
	private static final int TALON_TRAJECTORY_TIME_MIN = 0;
	private static final int TALON_VOLTAGE_FORWARD_MAX = 12;
	private static final int TALON_VOLTAGE_FORWARD_MIN = 0;
	private static final int TALON_VOLTAGE_REVERSE_MAX = 0;
	private static final int TALON_VOLTAGE_REVERSE_MIN = -12;
	private PIDSourceType m_pidSource = PIDSourceType.kDisplacement;
	private long talonJNIInstanceID;

	//Throwables
	private class AttemptedPIDWithIncorrectControlModeException extends UnsupportedOperationException {
		AttemptedPIDWithIncorrectControlModeException(String methodName) {
			super("Attempted to call the " + methodName + " method of a CAN " + type.name() + " on " + controlMode.name() + " mode");
		}
	}
	private class ExtraParametersNotApplicableException extends IllegalArgumentException {
		ExtraParametersNotApplicableException(String methodName, String[] parameters) {
			super(type.name() + "s don't use the extra argument for " + methodName + "(..." + implodeStringArray(parameters) + ")");
		}
	}
	private class InvalidJaguarFirmwareVersionException extends IndexOutOfBoundsException {
		InvalidJaguarFirmwareVersionException(boolean isAboveMaximum) {
			super("Jaguar #" + m_deviceNumberJaguar + "'s firmware version (" + jagFirmwareVersion + ") is " + (isAboveMaximum ? "not FIRST approved (version number above highest approved version)" : "is too old (must be at least version 108 of the FIRST approved firmware)"));
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
		jagFirmwareVersion = 0;
		jagHardwareVersion = 0;
		jagReceivedStatusMessages = new boolean[]{false, false, false};
		jagVerifiedStatuses = new boolean[JaguarVerifiedStatuses.values().length];

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
			jagFirmwareVersion = unpackINT32(data);
			if (jagReceivedStatusMessages[0] && jagReceivedStatusMessages[1] && jagReceivedStatusMessages[2]) {
				break;
			}
		}

		if (jagReceivedStatusMessages[0] && jagReceivedStatusMessages[1] && jagReceivedStatusMessages[2]) {
			getMessage(520225088, JaguarCANMessageID.VerificationMask.getValue(), data);
			jagHardwareVersion = data[0];
			if (jagFirmwareVersion < JAGUAR_FIRMWARE_VERSION_MIN) {
				throw new InvalidJaguarFirmwareVersionException(false);
			} else if (jagFirmwareVersion > JAGUAR_FIRMWARE_VERSION_MAX) {
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

	//NOTE Below - implements CANSpeedController, SpeedController, and PIDInterface

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
				if (m.isTalonOnly()) {
					throw new IllegalArgumentException("That control mode cannot be used on " + type.name() + "s!");
				}
				disableControl();
				controlMode = m;
				jagVerifiedStatuses[JaguarVerifiedStatuses.ControlMode.ordinal()] = false;
				break;
			case TalonSRX:
				if (controlMode != m) {
					applyControlMode(m);
				}
				break;
		}
	}

	/**
	 * Renamed -> isControlEnabled()
	 * @deprecated
	 */
	@Deprecated
	public boolean isEnabled() {
		return isControlEnabled;
	}

	public boolean isControlEnabled() {
		return isControlEnabled;
	}

	/**
	 * Renamed -> disableControl()
	 * @deprecated
	 */
	@Deprecated
	public void disable() {
		disableControl();
	}

	public void disableControl() {
		if (!isControlEnabled) return;
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

	/**
	 * Renamed -> enableControl()
	 * @deprecated
	 */
	@Deprecated
	public void enable() {
		enableControl();
	}

	public void enableControl() {
		if (isControlEnabled) return;
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
				if (jagVerifiedStatuses != JAGUAR_VERIFIED_REFERENCE) {
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
				Timer.delay(TALON_GET_PID_DELAY_S);
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
				jagVerifiedStatuses[JaguarVerifiedStatuses.P.ordinal()] = false;
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
				Timer.delay(TALON_GET_PID_DELAY_S);
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
				jagVerifiedStatuses[JaguarVerifiedStatuses.I.ordinal()] = false;
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
				Timer.delay(TALON_GET_PID_DELAY_S);
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
				jagVerifiedStatuses[JaguarVerifiedStatuses.D.ordinal()] = false;
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
		Timer.delay(TALON_GET_PID_DELAY_S);
		return CanTalonJNI.GetFgain(talonJNIInstanceID, m_profile);
	}

	public void setF(double f) {
		CanTalonJNI.SetFgain(talonJNIInstanceID, m_profile, f);
	}

	public double getIZone() {
		if (m_profile == 0) {
			CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot0_IZone.value);
		} else {
			CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot1_IZone.value);
		}
		Timer.delay(TALON_GET_PID_DELAY_S);
		return (double) CanTalonJNI.GetIzone(talonJNIInstanceID, m_profile);
	}

	public void setIZone(int iZone) {
		CanTalonJNI.SetIzone(talonJNIInstanceID, m_profile, iZone);
	}

	public double getCloseLoopRampRate() {
		if (m_profile == 0) {
			CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot0_CloseLoopRampRate.value);
		} else {
			CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot1_CloseLoopRampRate.value);
		}
		Timer.delay(TALON_GET_PID_DELAY_S);
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

	/**
	 * Provided by get()
	 * @deprecated
	 */
	@Deprecated
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

	/**
	 * Provided by set()
	 * @deprecated
	 */
	@Deprecated
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

	//NOTE Below - implements PIDOutput, PIDSource

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

	public PIDSourceType getPIDSourceType() {
		return m_pidSource;
	}

	public void setPIDSourceType(PIDSourceType pidSource) {
		m_pidSource = pidSource;
	}

	//NOTE Below - implements MotorSafety

	public double getExpiration() {
		return m_safetyHelper.getExpiration();
	}

	public void setExpiration(double timeout) {
		m_safetyHelper.setExpiration(timeout);
	}

	public boolean isAlive() {
		return m_safetyHelper.isAlive();
	}

	/**
	 * Provided by disableControl()
	 * @deprecated
	 */
	@Deprecated
	public void stopMotor() {
		disableControl();
	}

	public boolean isSafetyEnabled() {
		return m_safetyHelper.isSafetyEnabled();
	}

	public void setSafetyEnabled(boolean isEnabled) {
		m_safetyHelper.setSafetyEnabled(isEnabled);
	}

	public String getDescription() {
		return "GenericCANMotorController ID " + m_deviceNumberJaguar;
	}

	//NOTE Below - implements LiveWindowSendable, Sendable

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

	//NOTE Below - Jaguar and TalonSRX methods

	private static void sendMessageHelper(int messageID, byte[] data, int dataSize, int period) throws CANMessageNotFoundException {
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

	private void verify() throws CANMessageNotFoundException {
		byte[] data = new byte[8];
		try {
			getMessage(JaguarCANMessageID.verify_Init.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
			if (data[0] != 0) {
				data[0] = 1;
				sendMessage(JaguarCANMessageID.verify_Init.getValue(), data, 1);
				jagVerifiedStatuses = new boolean[]{
						false, false, false,
						false, false,
						false, false,
						false, false, false,
						false, false, false, false, false
				};
				jagReceivedStatusMessages[0] = false;
				jagReceivedStatusMessages[1] = false;
				jagReceivedStatusMessages[2] = false;
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

		if (isControlEnabled && !jagVerifiedStatuses[JaguarVerifiedStatuses.ControlMode.ordinal()]) {
			try {
				getMessage(33691200, JaguarCANMessageID.VerificationMask.getValue(), data);
				if (controlMode == CANDeviceControlMode.values()[data[0]]) {
					jagVerifiedStatuses[JaguarVerifiedStatuses.ControlMode.ordinal()] = true;
				} else {
					enableControl();
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33691200);
			}
		}

		byte var28;
		if (!jagVerifiedStatuses[JaguarVerifiedStatuses.SpeedReference.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.setSpeedReference.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				var28 = data[0];
				if (m_speedReference == var28) {
					jagVerifiedStatuses[JaguarVerifiedStatuses.SpeedReference.ordinal()] = true;
				} else {
					setSpeedReference(m_speedReference);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.setSpeedReference.getValue());
			}
		}
		if (!jagVerifiedStatuses[JaguarVerifiedStatuses.PositionReference.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.setPositionReference.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				var28 = data[0];
				if (m_positionReference == var28) {
					jagVerifiedStatuses[JaguarVerifiedStatuses.PositionReference.ordinal()] = true;
				} else {
					setPositionReference(m_positionReference);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.setPositionReference.getValue());
			}
		}

		int var29;
		double var30;
		if (!jagVerifiedStatuses[JaguarVerifiedStatuses.P.ordinal()]) {
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
					jagVerifiedStatuses[JaguarVerifiedStatuses.P.ordinal()] = true;
				} else {
					setP(m_p);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(var29);
			}
		}
		if (!jagVerifiedStatuses[JaguarVerifiedStatuses.I.ordinal()]) {
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
					jagVerifiedStatuses[JaguarVerifiedStatuses.I.ordinal()] = true;
				} else {
					setI(m_i);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(var29);
			}
		}
		if (!jagVerifiedStatuses[JaguarVerifiedStatuses.D.ordinal()]) {
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
					jagVerifiedStatuses[JaguarVerifiedStatuses.D.ordinal()] = true;
				} else {
					setD(m_d);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(var29);
			}
		}

		if (!jagVerifiedStatuses[JaguarVerifiedStatuses.NeutralMode.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.configNeutralMode.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				if (NeutralMode.valueOf(data[0]) == m_neutralMode) {
					jagVerifiedStatuses[JaguarVerifiedStatuses.NeutralMode.ordinal()] = true;
				} else {
					configNeutralMode(m_neutralMode);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.configNeutralMode.getValue());
			}
		}

		short var32;
		if (!jagVerifiedStatuses[JaguarVerifiedStatuses.EncoderCodesPerRevolution.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.configEncoderCodesPerRev.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				var32 = unpackINT16(data);
				if (var32 == m_encoderCodesPerRev) {
					jagVerifiedStatuses[JaguarVerifiedStatuses.EncoderCodesPerRevolution.ordinal()] = true;
				} else {
					configEncoderCodesPerRev(m_encoderCodesPerRev);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.configEncoderCodesPerRev.getValue());
			}
		}
		if (!jagVerifiedStatuses[JaguarVerifiedStatuses.PotentiometerTurns.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.configPotentiometerTurns.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				var32 = unpackINT16(data);
				if (var32 == m_potentiometerTurns) {
					jagVerifiedStatuses[JaguarVerifiedStatuses.PotentiometerTurns.ordinal()] = true;
				} else {
					configPotentiometerTurns(m_potentiometerTurns);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.configPotentiometerTurns.getValue());
			}
		}

		if (!jagVerifiedStatuses[JaguarVerifiedStatuses.LimitMode.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.configLimitMode.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				LimitMode var33 = LimitMode.valueOf(data[0]);
				if (var33 == m_limitMode) {
					jagVerifiedStatuses[JaguarVerifiedStatuses.LimitMode.ordinal()] = true;
				} else {
					configLimitMode(m_limitMode);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.configLimitMode.getValue());
			}
		}

		double var34;
		if (!jagVerifiedStatuses[JaguarVerifiedStatuses.ForwardLimit.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.configForwardLimit.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				var34 = unpackFXP16_16(data);
				if (FXP16_EQ(var34, m_forwardLimit)) {
					jagVerifiedStatuses[JaguarVerifiedStatuses.ForwardLimit.ordinal()] = true;
				} else {
					configForwardLimit(m_forwardLimit);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.configForwardLimit.getValue());
			}
		}
		if (!jagVerifiedStatuses[JaguarVerifiedStatuses.ReverseLimit.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.configReverseLimit.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				var34 = unpackFXP16_16(data);
				if (FXP16_EQ(var34, m_reverseLimit)) {
					jagVerifiedStatuses[JaguarVerifiedStatuses.ReverseLimit.ordinal()] = true;
				} else {
					configReverseLimit(m_reverseLimit);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.configReverseLimit.getValue());
			}
		}
		if (!jagVerifiedStatuses[JaguarVerifiedStatuses.MaxOutputVoltage.ordinal()]) {
			try {
				getMessage(JaguarCANMessageID.configMaxOutputVoltage.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
				var34 = unpackFXP8_8(data);
				if (Math.abs(var34 - m_maxOutputVoltage) < 0.1) {
					jagVerifiedStatuses[JaguarVerifiedStatuses.MaxOutputVoltage.ordinal()] = true;
				} else {
					configMaxOutputVoltage(m_maxOutputVoltage);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.configMaxOutputVoltage.getValue());
			}
		}
		if (!jagVerifiedStatuses[JaguarVerifiedStatuses.VoltageRampRate.ordinal()]) {
			if (controlMode == CANDeviceControlMode.PercentVbus) {
				try {
					getMessage(JaguarCANMessageID.setVoltageRampRate_Percent.getValue(), JaguarCANMessageID.VerificationMask.getValue(), data);
					var34 = unpackPercentage(data);
					if (FXP16_EQ(var34, m_voltageRampRate)) {
						jagVerifiedStatuses[JaguarVerifiedStatuses.VoltageRampRate.ordinal()] = true;
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
					jagVerifiedStatuses[JaguarVerifiedStatuses.VoltageRampRate.ordinal()] = true;
				} else {
					setVoltageRampRate(m_voltageRampRate);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(JaguarCANMessageID.setVoltageRampRate_Voltage.getValue());
			}
		}

		if (!jagVerifiedStatuses[JaguarVerifiedStatuses.FaultTime.ordinal()]) {
			try {
				getMessage(33693184, JaguarCANMessageID.VerificationMask.getValue(), data);
				var32 = unpackINT16(data);
				if ((int)((double) m_faultTime * 1000) == var32) {
					jagVerifiedStatuses[JaguarVerifiedStatuses.FaultTime.ordinal()] = true;
				} else {
					configFaultTime(m_faultTime);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33693184);
			}
		}

		if (!jagReceivedStatusMessages[0] || !jagReceivedStatusMessages[1] || !jagReceivedStatusMessages[2]) {
			setupPeriodicStatus();
			getTemperature();
			getPosition();
			updatePeriodicStatus();
		}
	}

	private void setSpeedReference(int reference) {
		sendMessage(JaguarCANMessageID.setSpeedReference.getValue(), new byte[]{(byte) reference}, 1);
		m_speedReference = reference;
		jagVerifiedStatuses[JaguarVerifiedStatuses.SpeedReference.ordinal()] = false;
	}

	private void setPositionReference(int reference) {
		sendMessage(JaguarCANMessageID.setPositionReference.getValue(), new byte[]{(byte) reference}, 1);
		m_positionReference = reference;
		jagVerifiedStatuses[JaguarVerifiedStatuses.PositionReference.ordinal()] = false;
	}

	private void setPercentMode() {
		setControlMode(CANDeviceControlMode.PercentVbus);
		setPositionReference(255);
		setSpeedReference(255);
	}

	private void setPercentMode_Encoder(int codesPerRev, boolean isQuadEncoder) {
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

	private void setPercentMode_Potentiometer() {
		setControlMode(CANDeviceControlMode.PercentVbus);
		setPositionReference(1);
		setSpeedReference(255);
		configPotentiometerTurns(1);
	}

	private void setCurrentMode(double p, double i, double d) {
		setControlMode(CANDeviceControlMode.Current);
		setPositionReference(255);
		setSpeedReference(255);
		setPID(p, i, d);
	}

	private void setCurrentMode_Encoder(double p, double i, double d, int codesPerRev, boolean isQuadEncoder) {
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

	private void setCurrentMode_Potentiometer(double p, double i, double d) {
		setControlMode(CANDeviceControlMode.Current);
		setPositionReference(1);
		setSpeedReference(255);
		configPotentiometerTurns(1);
		setPID(p, i, d);
	}

	private void setSpeedMode(double p, double i, double d, int codesPerRev, boolean isQuadEncoder) {
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

	private void setPositionMode_QuadEncoder(double p, double i, double d, int codesPerRev) {
		setControlMode(CANDeviceControlMode.Position);
		setPositionReference(0);
		configEncoderCodesPerRev(codesPerRev);
		setPID(p, i, d);
	}

	private void setPositionMode_Potentiometer(double p, double i, double d) {
		setControlMode(CANDeviceControlMode.Position);
		setPositionReference(1);
		configPotentiometerTurns(1);
		setPID(p, i, d);
	}

	private void setVoltageMode() {
		setControlMode(CANDeviceControlMode.Voltage);
		setPositionReference(255);
		setSpeedReference(255);
	}

	private void setVoltageMode_Encoder(int codesPerRev, boolean isQuadEncoder) {
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

	private void setVoltageMode_Potentiometer() {
		setControlMode(CANDeviceControlMode.Voltage);
		setPositionReference(1);
		setSpeedReference(255);
		configPotentiometerTurns(1);
	}

	private void setPosition(double pos) {
		int nativePos = ScaleRotationsToNativeUnits(m_feedbackDevice, pos);
		CanTalonJNI.SetSensorPosition(talonJNIInstanceID, nativePos);
	}

	private boolean getForwardLimitOK() {
		updatePeriodicStatus();
		return (m_limits & 1) != 0;
	}

	private boolean getReverseLimitOK() {
		updatePeriodicStatus();
		return (m_limits & 2) != 0;
	}

	private int getFirmwareVersion() {
		switch (type) {
			case Jaguar:
				return jagFirmwareVersion;
			case TalonSRX:
				CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eFirmVers.value);
				Timer.delay(TALON_GET_PID_DELAY_S);
				return CanTalonJNI.GetParamResponseInt32(talonJNIInstanceID, CanTalonJNI.param_t.eFirmVers.value);
			default:
				return RobotKiller.intKill();
		}
	}

	private byte getHardwareVersion() {
		return jagHardwareVersion;
	}

	private void configNeutralMode(NeutralMode mode) {
		sendMessage(JaguarCANMessageID.configNeutralMode.getValue(), new byte[]{mode.value}, 1);
		m_neutralMode = mode;
		jagVerifiedStatuses[JaguarVerifiedStatuses.NeutralMode.ordinal()] = false;
	}

	private void configEncoderCodesPerRev(int codesPerRev) {
		switch (type) {
			case Jaguar:
				byte[] data = new byte[8];
				byte dataSize = packINT16(data, (short) codesPerRev);
				sendMessage(JaguarCANMessageID.configEncoderCodesPerRev.getValue(), data, dataSize);
				m_encoderCodesPerRev = (short) codesPerRev;
				jagVerifiedStatuses[JaguarVerifiedStatuses.EncoderCodesPerRevolution.ordinal()] = false;
				break;
			case TalonSRX:
				m_codesPerRev = codesPerRev;
				setParameter(CanTalonJNI.param_t.eNumberEncoderCPR, (double) m_codesPerRev);
				break;
		}
	}

	private void configPotentiometerTurns(int turns) {
		switch (type) {
			case Jaguar:
				byte[] data = new byte[8];
				byte dataSize = packINT16(data, (short) turns);
				sendMessage(JaguarCANMessageID.configPotentiometerTurns.getValue(), data, dataSize);
				m_potentiometerTurns = (short) turns;
				jagVerifiedStatuses[JaguarVerifiedStatuses.PotentiometerTurns.ordinal()] = false;
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
		jagVerifiedStatuses[JaguarVerifiedStatuses.ForwardLimit.ordinal()] = false;
	}

	private void configReverseLimit(double reverseLimitPosition) {
		byte[] data = new byte[8];
		byte dataSize = packFXP16_16(data, reverseLimitPosition);
		int dataSize1 = dataSize + 1;
		data[dataSize] = 1;
		sendMessage(JaguarCANMessageID.configReverseLimit.getValue(), data, dataSize1);
		m_reverseLimit = reverseLimitPosition;
		jagVerifiedStatuses[JaguarVerifiedStatuses.ReverseLimit.ordinal()] = false;
	}

	private void configMaxOutputVoltage(double voltage) {
		switch (type) {
			case Jaguar:
				byte[] data = new byte[8];
				byte dataSize = packFXP8_8(data, voltage);
				sendMessage(JaguarCANMessageID.configMaxOutputVoltage.getValue(), data, dataSize);
				m_maxOutputVoltage = voltage;
				jagVerifiedStatuses[JaguarVerifiedStatuses.MaxOutputVoltage.ordinal()] = false;
				break;
			case TalonSRX:
				configPeakOutputVoltage(voltage, -voltage);
				break;
		}
	}

	private void configFaultTime(float faultTime) {
		byte[] data = new byte[8];
		faultTime = Math.min(Math.max(faultTime, JAGUAR_FAULT_TIME_MIN), JAGUAR_FAULT_TIME_MAX);
		byte dataSize = packINT16(data, (short)((int)((double) faultTime * 1000)));
		sendMessage(33693184, data, dataSize);
		m_faultTime = faultTime;
		jagVerifiedStatuses[JaguarVerifiedStatuses.FaultTime.ordinal()] = false;
	}

	private void sendMessage(int messageID, byte[] data, int dataSize, int period) {
		sendMessageHelper(messageID | m_deviceNumberJaguar, data, dataSize, period);
	}
	private void sendMessage(int messageID, byte[] data, int dataSize) {
		sendMessage(messageID, data, dataSize, 0);
	}

	private void requestMessage(int messageID, int period) {
		sendMessageHelper(messageID | m_deviceNumberJaguar, null, 0, 0);
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
		jagReceivedStatusMessages[0] = true;

		getMessage(33692224, JaguarCANMessageID.VerificationMask.getValue(), data);
		m_position = unpackFXP16_16(new byte[]{data[0], data[1], data[2], data[3]});
		m_speed = unpackFXP16_16(new byte[]{data[4], data[5], data[6], data[7]});
		jagReceivedStatusMessages[1] = true;

		getMessage(33692288, JaguarCANMessageID.VerificationMask.getValue(), data);
		m_limits = data[0];
		jagReceivedStatusMessages[2] = true;
	}

	private boolean FXP8_EQ(double a, double b) {
		return Math.floor(a * 256) == Math.floor(b * 256);
	}

	private boolean FXP16_EQ(double a, double b) {
		return Math.floor(a * 65536) == Math.floor(b * 65536);
	}

	private int getDeviceID() {
		return m_deviceNumberJaguar;
	}

	private void delete() {
		disableControl();
		if (talonJNIInstanceID != 0) {
			CanTalonJNI.delete_CanTalonSRX(talonJNIInstanceID);
			talonJNIInstanceID = 0;
		}
	}

	private void reverseSensor(boolean flip) {
		CanTalonJNI.SetRevFeedbackSensor(talonJNIInstanceID, flip ? 1 : 0);
	}

	private void reverseOutput(boolean flip) {
		CanTalonJNI.SetRevMotDuringCloseLoopEn(talonJNIInstanceID, flip ? 1 : 0);
	}

	private int getEncPosition() {
		return CanTalonJNI.GetEncPosition(talonJNIInstanceID);
	}

	private void setEncPosition(int newPosition) {
		setParameter(CanTalonJNI.param_t.eEncPosition, (double) newPosition);
	}

	private int getEncVelocity() {
		return CanTalonJNI.GetEncVel(talonJNIInstanceID);
	}

	private int getPulseWidthPosition() {
		return CanTalonJNI.GetPulseWidthPosition(talonJNIInstanceID);
	}

	private void setPulseWidthPosition(int newPosition) {
		setParameter(CanTalonJNI.param_t.ePwdPosition, (double) newPosition);
	}

	private int getPulseWidthVelocity() {
		return CanTalonJNI.GetPulseWidthVelocity(talonJNIInstanceID);
	}

	private int getPulseWidthRiseToFallUs() {
		return CanTalonJNI.GetPulseWidthRiseToFallUs(talonJNIInstanceID);
	}

	private int getPulseWidthRiseToRiseUs() {
		return CanTalonJNI.GetPulseWidthRiseToRiseUs(talonJNIInstanceID);
	}

	private FeedbackDeviceStatus isSensorPresent(FeedbackDevice feedbackDevice) {
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

	private int getNumberOfQuadIdxRises() {
		return CanTalonJNI.GetEncIndexRiseEvents(talonJNIInstanceID);
	}

	private int getPinStateQuadA() {
		return CanTalonJNI.GetQuadApin(talonJNIInstanceID);
	}

	private int getPinStateQuadB() {
		return CanTalonJNI.GetQuadBpin(talonJNIInstanceID);
	}

	private int getPinStateQuadIdx() {
		return CanTalonJNI.GetQuadIdxpin(talonJNIInstanceID);
	}

	private void setAnalogPosition(int newPosition) {
		setParameter(CanTalonJNI.param_t.eAinPosition, (double) newPosition);
	}

	private int getAnalogInPosition() {
		return CanTalonJNI.GetAnalogInWithOv(talonJNIInstanceID);
	}

	private int getAnalogInRaw() {
		return getAnalogInPosition() & 1023;
	}

	private int getAnalogInVelocity() {
		return CanTalonJNI.GetAnalogInVel(talonJNIInstanceID);
	}

	private int getClosedLoopError() {
		return CanTalonJNI.GetCloseLoopErr(talonJNIInstanceID);
	}

	private void setAllowableClosedLoopErr(int allowableCloseLoopError) {
		if (m_profile == 0) {
			setParameter(CanTalonJNI.param_t.eProfileParamSlot0_AllowableClosedLoopErr, (double) allowableCloseLoopError);
		} else {
			setParameter(CanTalonJNI.param_t.eProfileParamSlot1_AllowableClosedLoopErr, (double) allowableCloseLoopError);
		}
	}

	private boolean isFwdLimitSwitchClosed() {
		return CanTalonJNI.GetLimitSwitchClosedFor(talonJNIInstanceID) == 0;
	}

	private boolean isRevLimitSwitchClosed() {
		return CanTalonJNI.GetLimitSwitchClosedRev(talonJNIInstanceID) == 0;
	}

	private boolean getBrakeEnableDuringNeutral() {
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

	private void setFeedbackDevice(FeedbackDevice device) {
		m_feedbackDevice = device;
		CanTalonJNI.SetFeedbackDeviceSelect(talonJNIInstanceID, device.value);
	}

	private void setStatusFrameRateMs(StatusFrameRate stateFrame, int periodMs) {
		CanTalonJNI.SetStatusFrameRate(talonJNIInstanceID, stateFrame.value, periodMs);
	}

	private long getIAccum() {
		CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.ePidIaccum.value);
		Timer.delay(TALON_GET_PID_DELAY_S);
		return (long) CanTalonJNI.GetParamResponseInt32(talonJNIInstanceID, CanTalonJNI.param_t.ePidIaccum.value);
	}

	private void setVoltageCompensationRampRate(double rampRate) {
		CanTalonJNI.SetVoltageCompensationRate(talonJNIInstanceID, rampRate / 1000);
	}

	private void setProfile(int profile) {
		if (profile != 0 && profile != 1) {
			throw new IllegalArgumentException("Talon PID profile must be 0 or 1.");
		} else {
			m_profile = profile;
			CanTalonJNI.SetProfileSlotSelect(talonJNIInstanceID, m_profile);
		}
	}

	private int getForwardSoftLimit() {
		return CanTalonJNI.GetForwardSoftLimit(talonJNIInstanceID);
	}

	private void setForwardSoftLimit(double forwardLimit) {
		int nativeLimitPos = ScaleRotationsToNativeUnits(m_feedbackDevice, forwardLimit);
		CanTalonJNI.SetForwardSoftLimit(talonJNIInstanceID, nativeLimitPos);
	}

	private void enableForwardSoftLimit(boolean enable) {
		CanTalonJNI.SetForwardSoftEnable(talonJNIInstanceID, enable ? 1 : 0);
	}

	private boolean isForwardSoftLimitEnabled() {
		return CanTalonJNI.GetForwardSoftEnable(talonJNIInstanceID) != 0;
	}

	private int getReverseSoftLimit() {
		return CanTalonJNI.GetReverseSoftLimit(talonJNIInstanceID);
	}

	private void setReverseSoftLimit(double reverseLimit) {
		int nativeLimitPos = ScaleRotationsToNativeUnits(m_feedbackDevice, reverseLimit);
		CanTalonJNI.SetReverseSoftLimit(talonJNIInstanceID, nativeLimitPos);
	}

	private void enableReverseSoftLimit(boolean enable) {
		CanTalonJNI.SetReverseSoftEnable(talonJNIInstanceID, enable ? 1 : 0);
	}

	private boolean isReverseSoftLimitEnabled() {
		return CanTalonJNI.GetReverseSoftEnable(talonJNIInstanceID) != 0;
	}

	private void configPeakOutputVoltage(double forwardVoltage, double reverseVoltage) {
		setParameter(CanTalonJNI.param_t.ePeakPosOutput, Math.min(Math.max(forwardVoltage, TALON_VOLTAGE_FORWARD_MIN), TALON_VOLTAGE_FORWARD_MAX) * 1023 / 12);
		setParameter(CanTalonJNI.param_t.ePeakNegOutput, Math.min(Math.max(reverseVoltage, TALON_VOLTAGE_REVERSE_MIN), TALON_VOLTAGE_REVERSE_MAX) * 1023 / 12);
	}

	private void configNominalOutputVoltage(double forwardVoltage, double reverseVoltage) {
		setParameter(CanTalonJNI.param_t.eNominalPosOutput, Math.min(Math.max(forwardVoltage, TALON_VOLTAGE_FORWARD_MIN), TALON_VOLTAGE_FORWARD_MAX) * 1023 / 12);
		setParameter(CanTalonJNI.param_t.eNominalNegOutput, Math.min(Math.max(reverseVoltage, TALON_VOLTAGE_REVERSE_MIN), TALON_VOLTAGE_REVERSE_MAX) * 1023 / 12);
	}

	private void setParameter(CanTalonJNI.param_t paramEnum, double value) {
		CanTalonJNI.SetParam(talonJNIInstanceID, paramEnum.value, value);
	}

	private double getParameter(CanTalonJNI.param_t paramEnum) {
		CanTalonJNI.RequestParam(talonJNIInstanceID, paramEnum.value);
		Timer.delay(TALON_GET_PID_DELAY_S);
		return CanTalonJNI.GetParamResponse(talonJNIInstanceID, paramEnum.value);
	}

	private void clearStickyFaults() {
		CanTalonJNI.ClearStickyFaults(talonJNIInstanceID);
	}

	private void enableLimitSwitch(boolean forward, boolean reverse) {
		int mask = 4 + (forward ? 2 : 0) + (reverse ? 1 : 0);
		CanTalonJNI.SetOverrideLimitSwitchEn(talonJNIInstanceID, mask);
	}

	private void ConfigFwdLimitSwitchNormallyOpen(boolean normallyOpen) {
		CanTalonJNI.SetParam(talonJNIInstanceID, CanTalonJNI.param_t.eOnBoot_LimitSwitch_Forward_NormallyClosed.value, normallyOpen ? 0 : 1);
	}

	private void ConfigRevLimitSwitchNormallyOpen(boolean normallyOpen) {
		CanTalonJNI.SetParam(talonJNIInstanceID, CanTalonJNI.param_t.eOnBoot_LimitSwitch_Reverse_NormallyClosed.value, normallyOpen ? 0 : 1);
	}

	private void enableBrakeMode(boolean brake) {
		CanTalonJNI.SetOverrideBrakeType(talonJNIInstanceID, brake ? 2 : 1);
	}

	private int getFaultOverTemp() {
		return CanTalonJNI.GetFault_OverTemp(talonJNIInstanceID);
	}

	private int getFaultUnderVoltage() {
		return CanTalonJNI.GetFault_UnderVoltage(talonJNIInstanceID);
	}

	private int getFaultForLim() {
		return CanTalonJNI.GetFault_ForLim(talonJNIInstanceID);
	}

	private int getFaultRevLim() {
		return CanTalonJNI.GetFault_RevLim(talonJNIInstanceID);
	}

	private int getFaultHardwareFailure() {
		return CanTalonJNI.GetFault_HardwareFailure(talonJNIInstanceID);
	}

	private int getFaultForSoftLim() {
		return CanTalonJNI.GetFault_ForSoftLim(talonJNIInstanceID);
	}

	private int getFaultRevSoftLim() {
		return CanTalonJNI.GetFault_RevSoftLim(talonJNIInstanceID);
	}

	private int getStickyFaultOverTemp() {
		return CanTalonJNI.GetStckyFault_OverTemp(talonJNIInstanceID);
	}

	private int getStickyFaultUnderVoltage() {
		return CanTalonJNI.GetStckyFault_UnderVoltage(talonJNIInstanceID);
	}

	private int getStickyFaultForLim() {
		return CanTalonJNI.GetStckyFault_ForLim(talonJNIInstanceID);
	}

	private int getStickyFaultRevLim() {
		return CanTalonJNI.GetStckyFault_RevLim(talonJNIInstanceID);
	}

	private int getStickyFaultForSoftLim() {
		return CanTalonJNI.GetStckyFault_ForSoftLim(talonJNIInstanceID);
	}

	private int getStickyFaultRevSoftLim() {
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

	private void enableZeroSensorPositionOnIndex(boolean enable, boolean risingEdge) {
		setParameter(CanTalonJNI.param_t.eQuadIdxPolarity, risingEdge ? 1 : 0);
		setParameter(CanTalonJNI.param_t.eClearPositionOnIdx, enable ? 1 : 0);
	}

	private void changeMotionControlFramePeriod(int periodMs) {
		CanTalonJNI.ChangeMotionControlFramePeriod(talonJNIInstanceID, periodMs);
	}

	private void clearMotionProfileTrajectories() {
		CanTalonJNI.ClearMotionProfileTrajectories(talonJNIInstanceID);
	}

	private int getMotionProfileTopLevelBufferCount() {
		return CanTalonJNI.GetMotionProfileTopLevelBufferCount(talonJNIInstanceID);
	}

	private boolean pushMotionProfileTrajectory(TrajectoryPoint trajectoryPoint) {
		if (isMotionProfileTopLevelBufferFull()) {
			return false;
		}
		int targetPos = ScaleRotationsToNativeUnits(m_feedbackDevice, trajectoryPoint.position);
		int targetVel = ScaleVelocityToNativeUnits(m_feedbackDevice, trajectoryPoint.velocity);
		int profileSlotSelect = trajectoryPoint.profileSlotSelect > 0 ? 1 : 0;
		int timeDurMs = Math.min(Math.max(trajectoryPoint.timeDurMs, TALON_TRAJECTORY_TIME_MIN), TALON_TRAJECTORY_TIME_MAX);
		CanTalonJNI.PushMotionProfileTrajectory(talonJNIInstanceID, targetPos, targetVel, profileSlotSelect, timeDurMs, trajectoryPoint.velocityOnly ? 1 : 0, trajectoryPoint.isLastPoint ? 1 : 0, trajectoryPoint.zeroPos ? 1 : 0);
		return true;
	}

	private boolean isMotionProfileTopLevelBufferFull() {
		return CanTalonJNI.IsMotionProfileTopLevelBufferFull(talonJNIInstanceID);
	}

	private void processMotionProfileBuffer() {
		CanTalonJNI.ProcessMotionProfileBuffer(talonJNIInstanceID);
	}

	private void getMotionProfileStatus() {
		CanTalonJNI.GetMotionProfileStatus(talonJNIInstanceID, this, motionProfileStatus);
	}

	private void setMotionProfileStatusFromJNI(int flags, int profileSlotSelect, int targPos, int targVel, int topBufferRem, int topBufferCnt, int btmBufferCnt, int outputEnable) {
		motionProfileStatus = new MotionProfileStatus(topBufferRem, topBufferCnt, btmBufferCnt, (flags & 2) > 0, (flags & 4) > 0, (flags & 1) > 0, SetValueMotionProfile.valueOf(outputEnable));
		motionProfileStatus.activePoint.isLastPoint = (flags & 8) > 0;
		motionProfileStatus.activePoint.velocityOnly = (flags & 16) > 0;
		motionProfileStatus.activePoint.position = ScaleNativeUnitsToRotations(m_feedbackDevice, targPos);
		motionProfileStatus.activePoint.velocity = ScaleNativeUnitsToRpm(m_feedbackDevice, (long) targVel);
		motionProfileStatus.activePoint.profileSlotSelect = profileSlotSelect;
		motionProfileStatus.activePoint.zeroPos = false;
		motionProfileStatus.activePoint.timeDurMs = 0;
	}

	private void clearMotionProfileHasUnderrun() {
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
}
