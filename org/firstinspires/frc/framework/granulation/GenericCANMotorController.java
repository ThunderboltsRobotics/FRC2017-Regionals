package org.firstinspires.frc.framework.granulation;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CANJaguar.JaguarControlMode;
import edu.wpi.first.wpilibj.CANJaguar.LimitMode;
import edu.wpi.first.wpilibj.CANJaguar.NeutralMode;
import edu.wpi.first.wpilibj.CANTalon.*;
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

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Clutter-free replacement for the CANJaguar and CANTalon (for the TalonSRX model) classes from WPILibJ v2016.0.0.
 * Creating an object to manipulate motor controllers? You probably want MotorController.
 * CANJaguar.java and CANTalon.java (both decompiled) total to < 2600 lines. This file is < 2300 lines.
 * @author FRC 4739 Thunderbolts Robotics
 * @see MotorController
 * @version 2016-07-13/00
 */
@SuppressWarnings({"SameParameterValue", "unused", "WeakerAccess"})
public class GenericCANMotorController implements MotorSafety, PIDOutput, PIDSource, CANSpeedController {
	//Global
	private RioCANID port;
	private MotorControllerType type;
	private boolean isInverted = false;

	//Jaguar
	private JaguarControlMode jaguarControlMode;

	//TalonSRX
	private TalonControlMode talonControlMode;
	private long talonJNIInstanceID;

	public class AttemptedPIDWithIncorrectControlModeException extends UnsupportedOperationException {
		public AttemptedPIDWithIncorrectControlModeException(String methodName, JaguarControlMode m) {
			super("Attempted to call the " + methodName + " method of a CAN Jaguar on " + m.name() + " mode");
		}
		public AttemptedPIDWithIncorrectControlModeException(String methodName, TalonControlMode m) {
			super("Attempted to call the " + methodName + " method of a CAN TalonSRX on " + m.name() + " mode");
		}
	}

	//from CANJaguar and CANTalon
	private static final double kDelayForSolicitedSignals = 0.004;
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
	private byte m_hardwareVersion;
	private int m_firmwareVersion;
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
	private boolean[] m_jaguarReceivedStatusMessages;
	private boolean[] m_jaguarVerifiedStatuses;

	//TODO merge - from CANJaguar and CANTalon
	private boolean isControlEnabled_Jaguar;
	private boolean isControlEnabled_Talon;
	private byte m_deviceNumberJaguar;
	private int m_deviceNumberTalon;
	private ITable m_tableJaguar;
	private ITable m_tableTalon;
	private ITableListener m_table_listenerJaguar;
	private ITableListener m_table_listenerTalon;

	public GenericCANMotorController(RioCANID p, MotorControllerType m) throws CANMessageNotFoundException, CANIsUnsupportedException {
		switch (m) {
			case Jaguar:
				jaguarConstructor(p.getIDNumber());
				break;
			case TalonSRX:
				talonConstructor(p.getIDNumber());
				talonJNIInstanceID = CanTalonJNI.new_CanTalonSRX(p.getIDNumber());
				break;
			default:
				throw new CANIsUnsupportedException(m);
		}
		port = p;
		type = m;
	}

	public GenericCANMotorController(RioCANID p, MotorControllerType m, int talonSRXCAN_controlPeriod_ms) throws CANMessageNotFoundException, CANIsUnsupportedException {
		switch (m) {
			case Jaguar:
				jaguarConstructor(p.getIDNumber());
				break;
			case TalonSRX:
				talonConstructor(p.getIDNumber());
				talonJNIInstanceID = CanTalonJNI.new_CanTalonSRX(p.getIDNumber(), talonSRXCAN_controlPeriod_ms);
				break;
			default:
				throw new CANIsUnsupportedException(m);
		}
		port = p;
		type = m;
	}
	public GenericCANMotorController(RioCANID p, MotorControllerType m, int talonSRXCAN_controlPeriod_ms, int talonSRXCANEnablePeriod_ms) throws CANMessageNotFoundException, CANIsUnsupportedException {
		switch (m) {
			case Jaguar:
				jaguarConstructor(p.getIDNumber());
				break;
			case TalonSRX:
				talonConstructor(p.getIDNumber());
				talonJNIInstanceID = CanTalonJNI.new_CanTalonSRX(p.getIDNumber(), talonSRXCAN_controlPeriod_ms, talonSRXCANEnablePeriod_ms);
				break;
			default:
				throw new CANIsUnsupportedException(m);
		}
		port = p;
		type = m;
	}

	static void sendMessageHelper(int messageID, byte[] data, int dataSize, int period) throws CANMessageNotFoundException {
		int[] kTrustedMessages = new int[]{33685760, 33685824, 33686976, 33687040, 33687872, 33687936, 33689024, 33689088, 33689984, 33690048};
		ByteBuffer byteBuffer;
		for (int kTrustedMessage : kTrustedMessages) {
			if ((536870848 & messageID) == kTrustedMessage) {
				if (dataSize > 6) {
					throw new RuntimeException("CAN message has too much data.");
				}
				byteBuffer = ByteBuffer.allocateDirect(dataSize + 2);
				//TODO see if these changes work
//				byteBuffer.put(0, (byte) 0);
//				byteBuffer.put(1, (byte) 0);
				byteBuffer.put((byte) 0);
				byteBuffer.put((byte) 0);
				for (byte b = 0; b < dataSize; b++) {
//					byteBuffer.put(b + 2, data[b]);
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

		short intValue = (short)((int)(value * 32767));
		swap16(intValue, buffer);
		return (byte) 2;
	}

	private static byte packFXP8_8(byte[] buffer, double value) {
		short intValue = (short) (value * 256);
		swap16(intValue, buffer);
		return (byte) 2;
	}

	private static byte packFXP16_16(byte[] buffer, double value) {
		int intValue = (int) (value * 65536);
		swap32(intValue, buffer);
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

	private void talonConstructor(int deviceNumber) {
		m_pidSource = PIDSourceType.kDisplacement;
		m_tableTalon = null;
		m_table_listenerTalon = null;
		m_deviceNumberTalon = deviceNumber;
		m_safetyHelper = new MotorSafetyHelper(this);
		isControlEnabled_Talon = true;
		m_profile = 0;
		m_setPoint = 0;
		m_codesPerRev = 0;
		m_numPotTurns = 0;
		m_feedbackDevice = FeedbackDevice.QuadEncoder;
		setProfile(m_profile);
		applyControlMode(TalonControlMode.PercentVbus);
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
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.ControlMode.ordinal()] = true;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.SpeedReference.ordinal()] = true;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.PositionReference.ordinal()] = true;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.P.ordinal()] = true;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.I.ordinal()] = true;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.D.ordinal()] = true;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.NeutralMode.ordinal()] = true;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.EncoderCodesPerRevolution.ordinal()] = true;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.PotentiometerTurns.ordinal()] = true;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.ForwardLimit.ordinal()] = true;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.ReverseLimit.ordinal()] = true;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.LimitMode.ordinal()] = true;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.MaxOutputVoltage.ordinal()] = true;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.VoltageRampRate.ordinal()] = true;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.FaultTime.ordinal()] = true;
		m_busVoltage = 0;
		m_outputVoltage = 0;
		m_outputCurrent = 0;
		m_temperature = 0;
		m_position = 0;
		m_speed = 0;
		m_limits = 0;
		m_firmwareVersion = 0;
		m_hardwareVersion = 0;
		m_jaguarReceivedStatusMessages = new boolean[]{false, false, false};
		m_jaguarVerifiedStatuses = new boolean[JaguarVerifiedStatuses.values().length];
		isControlEnabled_Jaguar = true;
		m_tableJaguar = null;
		m_table_listenerJaguar = null;

		try {
			allocated.allocate(deviceNumber - 1);
		} catch (CheckedAllocationException e) {
			throw new AllocationException("GenericCANMotorController device " + e.getMessage() + "(increment index by one)");
		}

		m_deviceNumberJaguar = (byte) deviceNumber;
		jaguarControlMode = JaguarControlMode.PercentVbus;
		m_safetyHelper = new MotorSafetyHelper(this);
		byte[] data = new byte[8];
		requestMessage(-2147483136);
		requestMessage(520225088);

		for (int e = 0; e < 50; ++e) {
			Timer.delay(0.001);
			setupPeriodicStatus();
			updatePeriodicStatus();
			getMessage(512, 536870911, data);
			m_firmwareVersion = unpackINT32(data);
			if (m_jaguarReceivedStatusMessages[0] && m_jaguarReceivedStatusMessages[1] && m_jaguarReceivedStatusMessages[2]) {
				break;
			}
		}

		if (m_jaguarReceivedStatusMessages[0] && m_jaguarReceivedStatusMessages[1] && m_jaguarReceivedStatusMessages[2]) {
			try {
				getMessage(520225088, 536870911, data);
				m_hardwareVersion = data[0];
			} catch (CANMessageNotFoundException e) {
				m_hardwareVersion = 0;
			}

			if (m_firmwareVersion >= 3330 || m_firmwareVersion < 108) {
				if (m_firmwareVersion < 3330) {
					DriverStation.reportError("Jag " + m_deviceNumberJaguar + " firmware " + m_firmwareVersion + " is too old (must be at least version 108 of the FIRST approved firmware)", false);
				} else {
					DriverStation.reportError("Jag " + m_deviceNumberJaguar + " firmware " + m_firmwareVersion + " is not FIRST approved (must be at least version 108 of the FIRST approved firmware)", false);
				}

			}
		} else {
			free();
			throw new CANMessageNotFoundException();
		}
	}

	public void free() {
		allocated.free(m_deviceNumberJaguar - 1);
		m_safetyHelper = null;
		int messageID;
		switch(jaguarControlMode.ordinal()) {
			case 1:
				messageID = m_deviceNumberJaguar | 33685824;
				break;
			case 2:
				messageID = m_deviceNumberJaguar | 33687040;
				break;
			case 3:
				messageID = m_deviceNumberJaguar | 33689088;
				break;
			case 4:
				messageID = m_deviceNumberJaguar | 33690048;
				break;
			case 5:
				messageID = m_deviceNumberJaguar | 33687936;
				break;
			default:
				return;
		}

		CANJNI.FRCNetworkCommunicationCANSessionMuxSendMessage(messageID, null, -1);
		configMaxOutputVoltage(12);
	}

	public double get() {
		switch (type) {
			case Jaguar:
				return m_value;
			case TalonSRX:
				switch (talonControlMode.ordinal()) {
					case 1:
					case 2:
					default:
						return (double) CanTalonJNI.GetAppliedThrottle(talonJNIInstanceID) / 1023;
					case 3:
						return getOutputVoltage_Talon();
					case 4:
						return ScaleNativeUnitsToRpm(m_feedbackDevice, (long) CanTalonJNI.GetSensorVelocity(talonJNIInstanceID));
					case 5:
						return ScaleNativeUnitsToRotations(m_feedbackDevice, CanTalonJNI.GetSensorPosition(talonJNIInstanceID));
					case 6:
						return getOutputCurrent_Talon();
				}
			default:
				return RobotKiller.doubleKill();
		}
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

	public void set(double outputValue, byte syncGroup) {
		switch (type) {
			case Jaguar:
				byte[] data = new byte[8];
				if (isControlEnabled_Jaguar) {
					int messageID;
					byte dataSize;
					switch (jaguarControlMode.ordinal()) {
						case 1:
							messageID = 33685824;
							dataSize = packPercentage(data, isInverted ? -outputValue : outputValue);
							break;
						case 2:
							messageID = 33687040;
							dataSize = packFXP16_16(data, isInverted ? -outputValue : outputValue);
							break;
						case 3:
							messageID = 33689088;
							dataSize = packFXP16_16(data, outputValue);
							break;
						case 4:
							messageID = 33690048;
							dataSize = packFXP8_8(data, outputValue);
							break;
						case 5:
							messageID = 33687936;
							dataSize = packFXP8_8(data, isInverted ? -outputValue : outputValue);
							break;
						default:
							return;
					}

					if (syncGroup != 0) {
						data[dataSize++] = syncGroup;
					}

					sendMessage(messageID, data, dataSize, 20);
					if (m_safetyHelper != null) {
						m_safetyHelper.feed();
					}
				}

				m_value = outputValue;
				verify();
				break;
			case TalonSRX:
				System.err.println("Talons don't use the extra argument for set(..., byte syncGroup)");
				set(outputValue);
			default:
		}
	}

	public void set(double value) {
		switch (type) {
			case Jaguar:
				set(value, (byte) 0);
				break;
			case TalonSRX:
				m_safetyHelper.feed();
				if (isControlEnabled_Talon) {
					m_setPoint = value;
					switch (talonControlMode.ordinal()) {
						case 1:
							CanTalonJNI.Set(talonJNIInstanceID, isInverted ? -value : value);
							break;
						case 2:
							CanTalonJNI.SetDemand(talonJNIInstanceID, (int) value);
							break;
						case 3:
							CanTalonJNI.SetDemand(talonJNIInstanceID, 256 * (int) (isInverted ? -value : value));
							break;
						case 4:
							CanTalonJNI.SetDemand(talonJNIInstanceID, ScaleVelocityToNativeUnits(m_feedbackDevice, isInverted ? -value : value));
							break;
						case 5:
							CanTalonJNI.SetDemand(talonJNIInstanceID, ScaleRotationsToNativeUnits(m_feedbackDevice, value));
							break;
						case 6:
							CanTalonJNI.SetDemand(talonJNIInstanceID, 1000 * (int) (isInverted ? -value : value));
							break;
						case 7:
							CanTalonJNI.SetDemand(talonJNIInstanceID, (int) value);
							break;
					}
					CanTalonJNI.SetModeSelect(talonJNIInstanceID, talonControlMode.value);
				}
				break;
		}
	}

	public void reset() {
		switch (type) {
			case Jaguar:
				set(m_value);
				disableControl();
				break;
			case TalonSRX:
				disable_Talon();
				clearIAccum();
				break;
		}
	}

	public boolean getInverted() {
		return isInverted;
	}

	public void setInverted(boolean isInverted) {
		this.isInverted = isInverted;
	}

	public void verify() throws CANMessageNotFoundException {
		byte[] data = new byte[8];

		try {
			getMessage(33691136, 536870911, data);
			boolean e = data[0] != 0;
			if (e) {
				data[0] = 1;
				sendMessage(33691136, data, 1);
				m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.ControlMode.ordinal()] = false;
				m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.SpeedReference.ordinal()] = false;
				m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.PositionReference.ordinal()] = false;
				m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.NeutralMode.ordinal()] = false;
				m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.EncoderCodesPerRevolution.ordinal()] = false;
				m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.PotentiometerTurns.ordinal()] = false;
				m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.ForwardLimit.ordinal()] = false;
				m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.ReverseLimit.ordinal()] = false;
				m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.LimitMode.ordinal()] = false;
				m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.MaxOutputVoltage.ordinal()] = false;
				m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.FaultTime.ordinal()] = false;
				if (jaguarControlMode != JaguarControlMode.PercentVbus && jaguarControlMode != JaguarControlMode.Voltage) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.P.ordinal()] = false;
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.I.ordinal()] = false;
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.D.ordinal()] = false;
				} else {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.VoltageRampRate.ordinal()] = false;
				}

				m_jaguarReceivedStatusMessages[0] = false;
				m_jaguarReceivedStatusMessages[1] = false;
				m_jaguarReceivedStatusMessages[2] = false;
				int[] e1 = new int[]{33686912, 33688960, 33686720, 33688768, 33689792, 33686784, 33688832, 33689856, 33686848, 33688896, 33689920, 33692736, 33692800, 33692864, 33692928, 33693056, 33693120, 33685696, 33687808, 33693184, 33692992};

				for (int message : e1) {
					getMessage(message, 536870911, data);
				}
			}
		} catch (CANMessageNotFoundException e) {
			requestMessage(33691136);
		}

		if (!m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.ControlMode.ordinal()] && isControlEnabled_Jaguar) {
			try {
				getMessage(33691200, 536870911, data);
				if (jaguarControlMode == JaguarControlMode.values()[data[0]]) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.ControlMode.ordinal()] = true;
				} else {
					enableControl();
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33691200);
			}
		}

		byte var28;
		if (!m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.SpeedReference.ordinal()]) {
			try {
				getMessage(33686912, 536870911, data);
				var28 = data[0];
				if (m_speedReference == var28) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.SpeedReference.ordinal()] = true;
				} else {
					setSpeedReference(m_speedReference);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33686912);
			}
		}

		if (!m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.PositionReference.ordinal()]) {
			try {
				getMessage(33688960, 536870911, data);
				var28 = data[0];
				if (m_positionReference == var28) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.PositionReference.ordinal()] = true;
				} else {
					setPositionReference(m_positionReference);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33688960);
			}
		}

		int var29;
		double var30;
		if (!m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.P.ordinal()]) {
			var29 = 0;
			switch(jaguarControlMode.ordinal()) {
				case 2:
					var29 = 33686720;
					break;
				case 3:
					var29 = 33688768;
					break;
				case 4:
					var29 = 33689792;
			}

			try {
				getMessage(var29, 536870911, data);
				var30 = unpackFXP16_16(data);
				if (FXP16_EQ(m_p, var30)) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.P.ordinal()] = true;
				} else {
					setP(m_p);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(var29);
			}
		}

		if (!m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.I.ordinal()]) {
			var29 = 0;
			switch(jaguarControlMode.ordinal()) {
				case 2:
					var29 = 33686784;
					break;
				case 3:
					var29 = 33688832;
					break;
				case 4:
					var29 = 33689856;
			}

			try {
				getMessage(var29, 536870911, data);
				var30 = unpackFXP16_16(data);
				if (FXP16_EQ(m_i, var30)) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.I.ordinal()] = true;
				} else {
					setI(m_i);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(var29);
			}
		}

		if (!m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.D.ordinal()]) {
			var29 = 0;
			switch(jaguarControlMode.ordinal()) {
				case 2:
					var29 = 33686848;
					break;
				case 3:
					var29 = 33688896;
					break;
				case 4:
					var29 = 33689920;
			}

			try {
				getMessage(var29, 536870911, data);
				var30 = unpackFXP16_16(data);
				if (FXP16_EQ(m_d, var30)) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.D.ordinal()] = true;
				} else {
					setD(m_d);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(var29);
			}
		}

		if (!m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.NeutralMode.ordinal()]) {
			try {
				getMessage(33692864, 536870911, data);
				if (NeutralMode.valueOf(data[0]) == m_neutralMode) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.NeutralMode.ordinal()] = true;
				} else {
					configNeutralMode(m_neutralMode);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33692864);
			}
		}

		short var32;
		if (!m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.EncoderCodesPerRevolution.ordinal()]) {
			try {
				getMessage(33692736, 536870911, data);
				var32 = unpackINT16(data);
				if (var32 == m_encoderCodesPerRev) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.EncoderCodesPerRevolution.ordinal()] = true;
				} else {
					configEncoderCodesPerRev(m_encoderCodesPerRev);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33692736);
			}
		}

		if (!m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.PotentiometerTurns.ordinal()]) {
			try {
				getMessage(33692800, 536870911, data);
				var32 = unpackINT16(data);
				if (var32 == m_potentiometerTurns) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.PotentiometerTurns.ordinal()] = true;
				} else {
					configPotentiometerTurns(m_potentiometerTurns);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33692800);
			}
		}

		if (!m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.LimitMode.ordinal()]) {
			try {
				getMessage(33692928, 536870911, data);
				LimitMode var33 = LimitMode.valueOf(data[0]);
				if (var33 == m_limitMode) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.LimitMode.ordinal()] = true;
				} else {
					configLimitMode(m_limitMode);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33692928);
			}
		}

		double var34;
		if (!m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.ForwardLimit.ordinal()]) {
			try {
				getMessage(33692992, 536870911, data);
				var34 = unpackFXP16_16(data);
				if (FXP16_EQ(var34, m_forwardLimit)) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.ForwardLimit.ordinal()] = true;
				} else {
					configForwardLimit(m_forwardLimit);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33692992);
			}
		}

		if (!m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.ReverseLimit.ordinal()]) {
			try {
				getMessage(33693056, 536870911, data);
				var34 = unpackFXP16_16(data);
				if (FXP16_EQ(var34, m_reverseLimit)) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.ReverseLimit.ordinal()] = true;
				} else {
					configReverseLimit(m_reverseLimit);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33693056);
			}
		}

		if (!m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.MaxOutputVoltage.ordinal()]) {
			try {
				getMessage(33693120, 536870911, data);
				var34 = unpackFXP8_8(data);
				if (Math.abs(var34 - m_maxOutputVoltage) < 0.1) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.MaxOutputVoltage.ordinal()] = true;
				} else {
					configMaxOutputVoltage(m_maxOutputVoltage);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33693120);
			}
		}

		if (!m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.VoltageRampRate.ordinal()]) {
			if (jaguarControlMode == JaguarControlMode.PercentVbus) {
				try {
					getMessage(33685696, 536870911, data);
					var34 = unpackPercentage(data);
					if (FXP16_EQ(var34, m_voltageRampRate)) {
						m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.VoltageRampRate.ordinal()] = true;
					} else {
						setVoltageRampRate(m_voltageRampRate);
					}
				} catch (CANMessageNotFoundException e) {
					requestMessage(33685696);
				}
			}
		} else if (jaguarControlMode == JaguarControlMode.Voltage) {
			try {
				getMessage(33687808, 536870911, data);
				var34 = unpackFXP8_8(data);
				if (FXP8_EQ(var34, m_voltageRampRate)) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.VoltageRampRate.ordinal()] = true;
				} else {
					setVoltageRampRate(m_voltageRampRate);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33687808);
			}
		}

		if (!m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.FaultTime.ordinal()]) {
			try {
				getMessage(33693184, 536870911, data);
				var32 = unpackINT16(data);
				if ((int)((double) m_faultTime * 1000) == var32) {
					m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.FaultTime.ordinal()] = true;
				} else {
					configFaultTime(m_faultTime);
				}
			} catch (CANMessageNotFoundException e) {
				requestMessage(33693184);
			}
		}

		if (!m_jaguarReceivedStatusMessages[0] || !m_jaguarReceivedStatusMessages[1] || !m_jaguarReceivedStatusMessages[2]) {
			setupPeriodicStatus();
			getTemperature();
			getPosition();
			getFaults();
		}

	}

	/** @deprecated */
	@Deprecated
	public void disable() {
		disableControl();
	}

	public void enable() {
		enableControl();
	}

	public void pidWrite(double output) {
		switch (type) {
			case Jaguar:
				if (jaguarControlMode == JaguarControlMode.PercentVbus) {
					set(output);
				} else {
					//TODO check available modes
					throw new AttemptedPIDWithIncorrectControlModeException("pidWrite", jaguarControlMode);
				}
				break;
			case TalonSRX:
				if (getControlMode_Talon() == TalonControlMode.PercentVbus) {
					set(output);
				} else {
					//TODO check available modes
					throw new AttemptedPIDWithIncorrectControlModeException("pidWrite", talonControlMode);
				}
				break;
		}
	}

	private void setSpeedReference(int reference) {
		sendMessage(33686912, new byte[]{(byte) reference}, 1);
		m_speedReference = reference;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.SpeedReference.ordinal()] = false;
	}

	private void setPositionReference(int reference) {
		sendMessage(33688960, new byte[]{(byte) reference}, 1);
		m_positionReference = reference;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.PositionReference.ordinal()] = false;
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
			case Jaguar:
				System.err.println("Jaguars don't use the extra argument for set(..., double f, int iZone, double closeLoopRampRate, int profile)");
				setPID(p, i, d);
		}
	}

	public double getP() {
		switch (type) {
			case Jaguar:
				switch (jaguarControlMode) {
					case Current:
					case Speed:
					case Position:
						return m_p;
					case PercentVbus:
					case Voltage:
					default:
						throw new IllegalStateException("PID does not apply in Percent or Voltage control modes");
				}
			case TalonSRX:
				if (m_profile == 0) {
					CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot0_P.value);
				} else {
					CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot1_P.value);
				}

				Timer.delay(kDelayForSolicitedSignals);
				return CanTalonJNI.GetPgain(talonJNIInstanceID, m_profile);
			default:
				return RobotKiller.doubleKill();
		}
	}

	public void setP(double p) {
		switch (type) {
			case Jaguar:
				byte[] data = new byte[8];
				byte dataSize = packFXP16_16(data, p);
				switch(jaguarControlMode) {
					case Current:
						sendMessage(33686720, data, dataSize);
						break;
					case Speed:
						sendMessage(33688768, data, dataSize);
						break;
					case Position:
						sendMessage(33689792, data, dataSize);
						break;
					default:
						throw new IllegalStateException("PID constants only apply in Speed, Position, and Current mode");
				}

				m_p = p;
				m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.P.ordinal()] = false;
				break;
			case TalonSRX:
				CanTalonJNI.SetPgain(talonJNIInstanceID, m_profile, p);
				break;
		}
	}

	public double getI() {
		switch (type) {
			case Jaguar:
				switch (jaguarControlMode) {
					case Current:
					case Speed:
					case Position:
						return m_i;
					default:
						throw new IllegalStateException("PID does not apply in Percent or Voltage control modes");
				}
			case TalonSRX:
				if (m_profile == 0) {
					CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot0_I.value);
				} else {
					CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot1_I.value);
				}
				Timer.delay(kDelayForSolicitedSignals);
				return CanTalonJNI.GetIgain(talonJNIInstanceID, m_profile);
			default:
				return RobotKiller.doubleKill();
		}
	}

	public void setI(double i) {
		switch (type) {
			case Jaguar:
				byte[] data = new byte[8];
				byte dataSize = packFXP16_16(data, i);
				switch(jaguarControlMode) {
					case Current:
						sendMessage(33686784, data, dataSize);
						break;
					case Speed:
						sendMessage(33688832, data, dataSize);
						break;
					case Position:
						sendMessage(33689856, data, dataSize);
						break;
					default:
						throw new IllegalStateException("PID constants only apply in Speed, Position, and Current mode");
				}

				m_i = i;
				m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.I.ordinal()] = false;
				break;
			case TalonSRX:
				CanTalonJNI.SetIgain(talonJNIInstanceID, m_profile, i);
				break;
		}

	}

	//TODO merge
	public double getD() {
		if (!jaguarControlMode.equals(JaguarControlMode.PercentVbus) && !jaguarControlMode.equals(JaguarControlMode.Voltage)) {
			return m_d;
		} else {
			throw new IllegalStateException("PID does not apply in Percent or Voltage control modes");
		}
	}

	public void setD(double d) {
		switch (type) {
			case Jaguar:
				byte[] data = new byte[8];
				byte dataSize = packFXP16_16(data, d);
				switch(jaguarControlMode) {
					case Current:
						sendMessage(33686848, data, dataSize);
						break;
					case Speed:
						sendMessage(33688896, data, dataSize);
						break;
					case Position:
						sendMessage(33689920, data, dataSize);
						break;
					default:
						throw new IllegalStateException("PID constants only apply in Speed, Position, and Current mode");
				}

				m_d = d;
				m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.D.ordinal()] = false;
				break;
			case TalonSRX:
				CanTalonJNI.SetDgain(talonJNIInstanceID, m_profile, d);
				break;
		}
	}

	public double getD_Talon() {
		if (m_profile == 0) {
			CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot0_D.value);
		} else {
			CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot1_D.value);
		}

		Timer.delay(kDelayForSolicitedSignals);
		return CanTalonJNI.GetDgain(talonJNIInstanceID, m_profile);
	}

	public void enableControl(double encoderInitialPosition) {
		switch(jaguarControlMode.ordinal()) {
			case 1:
				sendMessage(33685760, new byte[0], 0);
				break;
			case 2:
				sendMessage(33686976, new byte[0], 0);
				break;
			case 3:
				byte[] data = new byte[8];
				byte dataSize = packFXP16_16(data, encoderInitialPosition);
				sendMessage(33689024, data, dataSize);
				break;
			case 4:
				sendMessage(33689984, new byte[0], 0);
				break;
			case 5:
				sendMessage(33687872, new byte[0], 0);
		}

		isControlEnabled_Jaguar = true;
	}

	public void enableControl() {
		enableControl(0);
	}

	public boolean isEnabled() {
		switch (type) {
			case Jaguar:
				return isControlEnabled_Jaguar;
			case TalonSRX:
				return isControlEnabled();
			default:
				return false;
		}
	}

	public void disableControl() {
		sendMessage(33685568, new byte[0], 0);
		sendMessage(33686592, new byte[0], 0);
		sendMessage(33688640, new byte[0], 0);
		sendMessage(33689664, new byte[0], 0);
		sendMessage(33687616, new byte[0], 0);
		sendMessage(33685824, new byte[0], 0, -1);
		sendMessage(33687040, new byte[0], 0, -1);
		sendMessage(33689088, new byte[0], 0, -1);
		sendMessage(33690048, new byte[0], 0, -1);
		sendMessage(33687936, new byte[0], 0, -1);
		isControlEnabled_Jaguar = false;
	}

	public void configEncoderCodesPerRev_Talon(int codesPerRev) {
		m_codesPerRev = codesPerRev;
		setParameter(CanTalonJNI.param_t.eNumberEncoderCPR, (double) m_codesPerRev);
	}

	public void configPotentiometerTurns_Talon(int turns) {
		m_numPotTurns = turns;
		setParameter(CanTalonJNI.param_t.eNumberPotTurns, (double) m_numPotTurns);
	}

	public void setPercentMode() {
		changeControlMode(JaguarControlMode.PercentVbus);
		setPositionReference(255);
		setSpeedReference(255);
	}

	public void setPercentMode_Encoder(int codesPerRev, boolean isQuadEncoder) {
		changeControlMode(JaguarControlMode.PercentVbus);
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
		changeControlMode(JaguarControlMode.PercentVbus);
		setPositionReference(1);
		setSpeedReference(255);
		configPotentiometerTurns(1);
	}

	public void setCurrentMode(double p, double i, double d) {
		changeControlMode(JaguarControlMode.Current);
		setPositionReference(255);
		setSpeedReference(255);
		setPID(p, i, d);
	}

	public void setCurrentMode_Encoder(double p, double i, double d, int codesPerRev, boolean isQuadEncoder) {
		changeControlMode(JaguarControlMode.Current);
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
		changeControlMode(JaguarControlMode.Current);
		setPositionReference(1);
		setSpeedReference(255);
		configPotentiometerTurns(1);
		setPID(p, i, d);
	}

	public void setSpeedMode(double p, double i, double d, int codesPerRev, boolean isQuadEncoder) {
		changeControlMode(JaguarControlMode.Speed);
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
		changeControlMode(JaguarControlMode.Position);
		setPositionReference(0);
		configEncoderCodesPerRev(codesPerRev);
		setPID(p, i, d);
	}

	public void setPositionMode_Potentiometer(double p, double i, double d) {
		changeControlMode(JaguarControlMode.Position);
		setPositionReference(1);
		configPotentiometerTurns(1);
		setPID(p, i, d);
	}

	public void setVoltageMode() {
		changeControlMode(JaguarControlMode.Voltage);
		setPositionReference(255);
		setSpeedReference(255);
	}

	public void setVoltageMode_Encoder(int codesPerRev, boolean isQuadEncoder) {
		changeControlMode(JaguarControlMode.Voltage);
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
		changeControlMode(JaguarControlMode.Voltage);
		setPositionReference(1);
		setSpeedReference(255);
		configPotentiometerTurns(1);
	}

	public ControlMode getControlMode() {
		//TODO throw exception
		return null;
	}

	public JaguarControlMode getControlMode_Jaguar() {
		return jaguarControlMode;
	}

	public TalonControlMode getControlMode_Talon() {
		return talonControlMode;
	}

	//TODO merge
	public void setControlMode(int mode) {
		changeControlMode(JaguarControlMode.values()[mode]);
	}
	public void setControlMode_Talon(int mode) {
		changeControlMode(TalonControlMode.valueOf(mode));
	}
	private void changeControlMode(JaguarControlMode controlMode) {
		disableControl();
		jaguarControlMode = controlMode;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.ControlMode.ordinal()] = false;
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

	//TODO merge
	public double getOutputVoltage() {
		updatePeriodicStatus();
		return m_outputVoltage;
	}

	public double getOutputVoltage_Talon() {
		return getBusVoltage() * (double) CanTalonJNI.GetAppliedThrottle(talonJNIInstanceID) / 1023;
	}

	//TODO merge
	public double getOutputCurrent() {
		updatePeriodicStatus();
		return m_outputCurrent;
	}

	public double getOutputCurrent_Talon() {
		return CanTalonJNI.GetCurrent(talonJNIInstanceID);
	}

	//TODO merge
	public double getTemperature() {
		updatePeriodicStatus();
		return m_temperature;
	}

	public double getTemperature_Talon() {
		return CanTalonJNI.GetTemp(talonJNIInstanceID);
	}

	//TODO merge
	public double getPosition() {
		updatePeriodicStatus();
		return m_position;
	}

	public void setPosition(double pos) {
		int nativePos = ScaleRotationsToNativeUnits(m_feedbackDevice, pos);
		CanTalonJNI.SetSensorPosition(talonJNIInstanceID, nativePos);
	}

	public double getPosition_Talon() {
		return ScaleNativeUnitsToRotations(m_feedbackDevice, CanTalonJNI.GetSensorPosition(talonJNIInstanceID));
	}

	//TODO merge
	public double getSpeed() {
		updatePeriodicStatus();
		return m_speed;
	}

	public double getSpeed_Talon() {
		return ScaleNativeUnitsToRpm(m_feedbackDevice, (long) CanTalonJNI.GetSensorVelocity(talonJNIInstanceID));
	}

	public boolean getForwardLimitOK() {
		updatePeriodicStatus();
		return (m_limits & 1) != 0;
	}

	public boolean getReverseLimitOK() {
		updatePeriodicStatus();
		return (m_limits & 2) != 0;
	}

	public void getFaults() {
		updatePeriodicStatus();
	}

	//TODO merge
	public void setVoltageRampRate(double rampRate) {
		byte[] data = new byte[8];
		byte dataSize;
		int message;
		switch(jaguarControlMode.ordinal()) {
			case 1:
				dataSize = packPercentage(data, rampRate / (m_maxOutputVoltage * 1000));
				message = 33685696;
				break;
			case 5:
				dataSize = packFXP8_8(data, rampRate / 1000);
				message = 33687808;
				break;
			default:
				throw new IllegalStateException("Voltage ramp rate only applies in Percentage and Voltage modes");
		}

		sendMessage(message, data, dataSize);
	}

	public void setVoltageRampRate_Talon(double rampRate) {
		int rate = (int)(rampRate * 1023 / 12 / 100);
		CanTalonJNI.SetRampThrottle(talonJNIInstanceID, rate);
	}

	//TODO merge
	public int getFirmwareVersion_Jaguar() {
		return m_firmwareVersion;
	}

	public long getFirmwareVersion_Talon() {
		CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eFirmVers.value);
		Timer.delay(kDelayForSolicitedSignals);
		return (long) CanTalonJNI.GetParamResponseInt32(talonJNIInstanceID, CanTalonJNI.param_t.eFirmVers.value);
	}

	public byte getHardwareVersion() {
		return m_hardwareVersion;
	}

	public void configNeutralMode(NeutralMode mode) {
		sendMessage(33692864, new byte[]{mode.value}, 1);
		m_neutralMode = mode;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.NeutralMode.ordinal()] = false;
	}

	public void configEncoderCodesPerRev(int codesPerRev) {
		byte[] data = new byte[8];
		byte dataSize = packINT16(data, (short) codesPerRev);
		sendMessage(33692736, data, dataSize);
		m_encoderCodesPerRev = (short) codesPerRev;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.EncoderCodesPerRevolution.ordinal()] = false;
	}

	public void configPotentiometerTurns(int turns) {
		byte[] data = new byte[8];
		byte dataSize = packINT16(data, (short) turns);
		sendMessage(33692800, data, dataSize);
		m_potentiometerTurns = (short) turns;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.PotentiometerTurns.ordinal()] = false;
	}

	public void configSoftPositionLimits(double forwardLimitPosition, double reverseLimitPosition) {
		configLimitMode(LimitMode.SoftPositionLimits);
		configForwardLimit(forwardLimitPosition);
		configReverseLimit(reverseLimitPosition);
	}

	public void disableSoftPositionLimits() {
		configLimitMode(LimitMode.SwitchInputsOnly);
	}

	public void configLimitMode(LimitMode mode) {
		sendMessage(33692928, new byte[]{mode.value}, 1);
	}

	public void configForwardLimit(double forwardLimitPosition) {
		byte[] data = new byte[8];
		byte dataSize = packFXP16_16(data, forwardLimitPosition);
		int dataSize1 = dataSize + 1;
		data[dataSize] = 1;
		sendMessage(33692992, data, dataSize1);
		m_forwardLimit = forwardLimitPosition;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.ForwardLimit.ordinal()] = false;
	}

	public void configReverseLimit(double reverseLimitPosition) {
		byte[] data = new byte[8];
		byte dataSize = packFXP16_16(data, reverseLimitPosition);
		int dataSize1 = dataSize + 1;
		data[dataSize] = 1;
		sendMessage(33693056, data, dataSize1);
		m_reverseLimit = reverseLimitPosition;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.ReverseLimit.ordinal()] = false;
	}

	public void configMaxOutputVoltage(double voltage) {
		byte[] data = new byte[8];
		byte dataSize = packFXP8_8(data, voltage);
		sendMessage(33693120, data, dataSize);
		m_maxOutputVoltage = voltage;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.MaxOutputVoltage.ordinal()] = false;
	}

	public void configFaultTime(float faultTime) {
		byte[] data = new byte[8];
		if (faultTime < 0.5) {
			faultTime = (float) 0.5;
		} else if (faultTime > 3) {
			faultTime = 3;
		}

		byte dataSize = packINT16(data, (short)((int)((double) faultTime * 1000)));
		sendMessage(33693184, data, dataSize);
		m_faultTime = faultTime;
		m_jaguarVerifiedStatuses[JaguarVerifiedStatuses.FaultTime.ordinal()] = false;
	}

	public void sendMessage(int messageID, byte[] data, int dataSize, int period) {
		sendMessageHelper(messageID | m_deviceNumberJaguar, data, dataSize, period);
	}

	public void sendMessage(int messageID, byte[] data, int dataSize) {
		sendMessage(messageID, data, dataSize, 0);
	}

	public void requestMessage(int messageID, int period) {
		sendMessageHelper(messageID | m_deviceNumberJaguar, null, 0, period);
	}

	public void requestMessage(int messageID) {
		requestMessage(messageID, 0);
	}

	public void getMessage(int messageID, int messageMask, byte[] data) throws CANMessageNotFoundException {
		messageID |= m_deviceNumberJaguar;
		messageID &= 536870911;
		ByteBuffer targetedMessageID = ByteBuffer.allocateDirect(4);
		targetedMessageID.order(ByteOrder.LITTLE_ENDIAN);
		targetedMessageID.asIntBuffer().put(0, messageID);
		ByteBuffer timeStamp = ByteBuffer.allocateDirect(4);
		ByteBuffer dataBuffer = CANJNI.FRCNetworkCommunicationCANSessionMuxReceiveMessage(targetedMessageID.asIntBuffer(), messageMask, timeStamp);
		if (data != null) {
			for (int i = 0; i < dataBuffer.capacity(); ++i) {
				data[i] = dataBuffer.get(i);
			}
		}

	}

	public void setupPeriodicStatus() {
		byte[] data = new byte[8];
		byte[] kMessage0Data = new byte[]{(byte) 3, (byte) 4, (byte) 1, (byte) 2, (byte) 5, (byte) 6, (byte) 7, (byte) 8};
		byte[] kMessage1Data = new byte[]{(byte) 9, (byte) 10, (byte) 11, (byte) 12, (byte) 13, (byte) 14, (byte) 15, (byte) 16};
		byte[] kMessage2Data = new byte[]{(byte) 18, (byte) 19, (byte) 0, (byte) 0, (byte) 0, (byte) 0, (byte) 0, (byte) 0};
		byte dataSize = packINT16(data, (short) 20);
		sendMessage(33691648, data, dataSize);
		sendMessage(33691712, data, dataSize);
		sendMessage(33691776, data, dataSize);
		byte dataSize1 = 8;
		sendMessage(33691904, kMessage0Data, dataSize1);
		sendMessage(33691968, kMessage1Data, dataSize1);
		sendMessage(33692032, kMessage2Data, dataSize1);
	}

	public void updatePeriodicStatus() throws CANMessageNotFoundException {
		byte[] data = new byte[8];

		getMessage(33692160, 536870911, data);
		m_busVoltage = unpackFXP8_8(new byte[]{data[0], data[1]});
		m_outputVoltage = unpackPercentage(new byte[]{data[2], data[3]}) * m_busVoltage;
		m_outputCurrent = unpackFXP8_8(new byte[]{data[4], data[5]});
		m_temperature = unpackFXP8_8(new byte[]{data[6], data[7]});
		m_jaguarReceivedStatusMessages[0] = true;

		getMessage(33692224, 536870911, data);
		m_position = unpackFXP16_16(new byte[]{data[0], data[1], data[2], data[3]});
		m_speed = unpackFXP16_16(new byte[]{data[4], data[5], data[6], data[7]});
		m_jaguarReceivedStatusMessages[1] = true;

		getMessage(33692288, 536870911, data);
		m_limits = data[0];
		m_jaguarReceivedStatusMessages[2] = true;

	}

	public boolean FXP8_EQ(double a, double b) {
		return (int)(a * 256) == (int)(b * 256);
	}

	public boolean FXP16_EQ(double a, double b) {
		return (int)(a * 65536) == (int)(b * 65536);
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

	public void initTable(ITable subtable) {
		m_tableJaguar = subtable;
		updateTable();
	}

	public ITable getTable() {
		return m_tableJaguar;
	}

	public void startLiveWindowMode() {
		switch (type) {
			case Jaguar:
				set(0);
				m_table_listenerJaguar = createTableListener();
				m_tableJaguar.addTableListener(m_table_listenerJaguar, true);
				break;
			case TalonSRX:
				set(0);
				m_table_listenerTalon = createTableListener();
				m_tableTalon.addTableListener(m_table_listenerTalon, true);
				break;
		}
	}

	public void stopLiveWindowMode() {
		switch (type) {
			case Jaguar:
				set(0);
				m_tableJaguar.removeTableListener(m_table_listenerJaguar);
				break;
			case TalonSRX:
				set(0);
				m_tableTalon.removeTableListener(m_table_listenerTalon);
				break;
		}
	}

	public PIDSourceType getPIDSourceType() {
		return m_pidSource;
	}

	public void setPIDSourceType(PIDSourceType pidSource) {
		m_pidSource = pidSource;
	}

	public double pidGet() {
		return getPosition_Talon();
	}

	public void delete() {
		disable_Talon();
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

	private void applyControlMode(TalonControlMode controlMode) {
		talonControlMode = controlMode;
		if (controlMode == TalonControlMode.Disabled) {
			isControlEnabled_Talon = false;
		}

		CanTalonJNI.SetModeSelect(talonJNIInstanceID, TalonControlMode.Disabled.value);
		UsageReporting.report(52, m_deviceNumberTalon + 1, controlMode.value);
	}

	public void changeControlMode(TalonControlMode controlMode) {
		if (talonControlMode != controlMode) {
			applyControlMode(controlMode);
		}

	}

	public void setFeedbackDevice(FeedbackDevice device) {
		m_feedbackDevice = device;
		CanTalonJNI.SetFeedbackDeviceSelect(talonJNIInstanceID, device.value);
	}

	public void setStatusFrameRateMs(StatusFrameRate stateFrame, int periodMs) {
		CanTalonJNI.SetStatusFrameRate(talonJNIInstanceID, stateFrame.value, periodMs);
	}

	public void enableControl_Talon() {
		changeControlMode(talonControlMode);
		isControlEnabled_Talon = true;
	}

	public void enable_Talon() {
		enableControl_Talon();
	}

	public void disableControl_Talon() {
		CanTalonJNI.SetModeSelect(talonJNIInstanceID, TalonControlMode.Disabled.value);
		isControlEnabled_Talon = false;
	}

	public boolean isControlEnabled() {
		return isControlEnabled_Talon;
	}

	public double getF() {
		if (m_profile == 0) {
			CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot0_F.value);
		} else {
			CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.eProfileParamSlot1_F.value);
		}

		Timer.delay(kDelayForSolicitedSignals);
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

		Timer.delay(kDelayForSolicitedSignals);
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

		Timer.delay(kDelayForSolicitedSignals);
		double throttlePerMs = (double) CanTalonJNI.GetCloseLoopRampRate(talonJNIInstanceID, m_profile);
		return throttlePerMs / 1023 * 12000;
	}

	public void setCloseLoopRampRate(double rampRate) {
		int rate = (int)(rampRate * 1023 / 12 / 1000);
		CanTalonJNI.SetCloseLoopRampRate(talonJNIInstanceID, m_profile, rate);
	}

	public long getIAccum() {
		CanTalonJNI.RequestParam(talonJNIInstanceID, CanTalonJNI.param_t.ePidIaccum.value);
		Timer.delay(kDelayForSolicitedSignals);
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
		disableControl_Talon();
	}

	public void disable_Talon() {
		disableControl_Talon();
	}

	public void clearIAccum() {
		CanTalonJNI.SetParam(talonJNIInstanceID, CanTalonJNI.param_t.ePidIaccum.value, 0);
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

	public void enableReverseSoftLimit(boolean enable) {
		CanTalonJNI.SetReverseSoftEnable(talonJNIInstanceID, enable ? 1 : 0);
	}

	public boolean isReverseSoftLimitEnabled() {
		return CanTalonJNI.GetReverseSoftEnable(talonJNIInstanceID) != 0;
	}

	public void configMaxOutputVoltage_Talon(double voltage) {
		configPeakOutputVoltage(voltage, -voltage);
	}

	public void configPeakOutputVoltage(double forwardVoltage, double reverseVoltage) {
		if (forwardVoltage > 12) {
			forwardVoltage = 12;
		} else if (forwardVoltage < 0) {
			forwardVoltage = 0;
		}

		if (reverseVoltage > 0) {
			reverseVoltage = 0;
		} else if (reverseVoltage < -12) {
			reverseVoltage = -12;
		}

		setParameter(CanTalonJNI.param_t.ePeakPosOutput, 1023 * forwardVoltage / 12);
		setParameter(CanTalonJNI.param_t.ePeakNegOutput, 1023 * reverseVoltage / 12);
	}

	public void configNominalOutputVoltage(double forwardVoltage, double reverseVoltage) {
		if (forwardVoltage > 12) {
			forwardVoltage = 12;
		} else if (forwardVoltage < 0) {
			forwardVoltage = 0;
		}

		if (reverseVoltage > 0) {
			reverseVoltage = 0;
		} else if (reverseVoltage < -12) {
			reverseVoltage = -12;
		}

		setParameter(CanTalonJNI.param_t.eNominalPosOutput, 1023 * forwardVoltage / 12);
		setParameter(CanTalonJNI.param_t.eNominalNegOutput, 1023 * reverseVoltage / 12);
	}

	public void setParameter(CanTalonJNI.param_t paramEnum, double value) {
		CanTalonJNI.SetParam(talonJNIInstanceID, paramEnum.value, value);
	}

	public double getParameter(CanTalonJNI.param_t paramEnum) {
		CanTalonJNI.RequestParam(talonJNIInstanceID, paramEnum.value);
		Timer.delay(kDelayForSolicitedSignals);
		return CanTalonJNI.GetParamResponse(talonJNIInstanceID, paramEnum.value);
	}

	public void clearStickyFaults() {
		CanTalonJNI.ClearStickyFaults(talonJNIInstanceID);
	}

	public void enableLimitSwitch(boolean forward, boolean reverse) {
		int mask = 4 + (forward ? 1 : 0) * 2 + (reverse ? 1 : 0);
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
		int toReturn = (int) fullRotations;
		double scalar = GetNativeUnitsPerRotationScalar(devToLookup);
		if (scalar > 0) {
			toReturn = (int)(fullRotations * scalar);
		}

		return toReturn;
	}

	private int ScaleVelocityToNativeUnits(FeedbackDevice devToLookup, double rpm) {
		int toReturn = (int) rpm;
		double scalar = GetNativeUnitsPerRotationScalar(devToLookup);
		if (scalar > 0) {
			toReturn = (int)(rpm / 600 * scalar);
		}

		return toReturn;
	}

	private double ScaleNativeUnitsToRotations(FeedbackDevice devToLookup, int nativePos) {
		double toReturn = (double) nativePos;
		double scalar = GetNativeUnitsPerRotationScalar(devToLookup);
		if (scalar > 0) {
			toReturn = (double) nativePos / scalar;
		}

		return toReturn;
	}

	private double ScaleNativeUnitsToRpm(FeedbackDevice devToLookup, long nativeVel) {
		double toReturn = (double) nativeVel;
		double scalar = GetNativeUnitsPerRotationScalar(devToLookup);
		if (scalar > 0) {
			toReturn = (double) nativeVel / (scalar / 600);
		}

		return toReturn;
	}

	public void enableZeroSensorPositionOnIndex(boolean enable, boolean risingEdge) {
		if (enable) {
			setParameter(CanTalonJNI.param_t.eQuadIdxPolarity, risingEdge ? 1 : 0);
			setParameter(CanTalonJNI.param_t.eClearPositionOnIdx, 1);
		} else {
			setParameter(CanTalonJNI.param_t.eClearPositionOnIdx, 0);
			setParameter(CanTalonJNI.param_t.eQuadIdxPolarity, risingEdge ? 1 : 0);
		}
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
		} else {
			int targetPos = ScaleRotationsToNativeUnits(m_feedbackDevice, trajectoryPoint.position);
			int targetVel = ScaleVelocityToNativeUnits(m_feedbackDevice, trajectoryPoint.velocity);
			int profileSlotSelect = trajectoryPoint.profileSlotSelect > 0 ? 1 : 0;
			int timeDurMs = trajectoryPoint.timeDurMs;
			if (timeDurMs > 255) {
				timeDurMs = 255;
			}

			if (timeDurMs < 0) {
				timeDurMs = 0;
			}

			CanTalonJNI.PushMotionProfileTrajectory(talonJNIInstanceID, targetPos, targetVel, profileSlotSelect, timeDurMs, trajectoryPoint.velocityOnly ? 1 : 0, trajectoryPoint.isLastPoint ? 1 : 0, trajectoryPoint.zeroPos ? 1 : 0);
			return true;
		}
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

	private enum JaguarVerifiedStatuses {
		EncoderCodesPerRevolution, PotentiometerTurns, MaxOutputVoltage, VoltageRampRate, FaultTime,
		ControlMode, LimitMode, NeutralMode,
		ForwardLimit, ReverseLimit,
		SpeedReference, PositionReference,
		P, I, D
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
