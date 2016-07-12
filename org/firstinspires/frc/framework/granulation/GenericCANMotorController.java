package org.firstinspires.frc.framework.granulation;

import edu.wpi.first.wpilibj.CANJaguar.JaguarControlMode;
import edu.wpi.first.wpilibj.CANJaguar.LimitMode;
import edu.wpi.first.wpilibj.CANJaguar.NeutralMode;
import edu.wpi.first.wpilibj.*;
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
import org.firstinspires.frc.framework.hardware.MotorController;
import org.firstinspires.frc.framework.hardware.MotorController.CANIsUnsupportedException;
import org.firstinspires.frc.framework.hardware.MotorControllerType;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Clutter-free replacement for the CANJaguar and CANTalon (for the TalonSRX model) classes from WPILibJ v2016.0.0.
 * Creating an object to manipulate motor controllers? You probably want MotorController.
 * CANJaguar.java and CANTalon.java (both decompiled) total to < 2600 lines. This file is < 2200 lines.
 * @author FRC 4739 Thunderbolts Robotics
 * @see MotorController
 * @version 2016-07-10/21
 */
@SuppressWarnings({"SameParameterValue", "unused", "WeakerAccess"})
public class GenericCANMotorController implements MotorSafety, PIDOutput, PIDSource, CANSpeedController {
	//mine
	private RioCANID port;
	private MotorControllerType type;

	//from CANJaguar and CANTalon
	private static final double kDelayForSolicitedSignals = 0.004;
	private static final Resource allocated = new Resource(63);

	private boolean isInverted = false;
	private double m_value = 0;
	private double m_p = 0;
	private double m_i = 0;
	private double m_d = 0;
	private int m_speedReference = 255;
	private int m_positionReference = 255;

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
	private long m_handle;
	private int m_firmwareVersion;
	private int m_profile;
	private int m_codesPerRev;
	private int m_numPotTurns;

	private FeedbackDevice m_feedbackDevice;
	private LimitMode m_limitMode;
	private MotionProfileStatus motionProfileStatus;
	private MotorSafetyHelper m_safetyHelper;
	private NeutralMode m_neutralMode;
	private PIDSourceType m_pidSource;

	//TODO work out this 'verified' system
	private boolean[] m_receivedStatusMessages;
	private boolean m_controlModeVerified;
	private boolean m_speedRefVerified;
	private boolean m_posRefVerified;
	private boolean m_pVerified;
	private boolean m_iVerified;
	private boolean m_dVerified;
	private boolean m_neutralModeVerified;
	private boolean m_encoderCodesPerRevVerified;
	private boolean m_potentiometerTurnsVerified;
	private boolean m_forwardLimitVerified;
	private boolean m_reverseLimitVerified;
	private boolean m_limitModeVerified;
	private boolean m_maxOutputVoltageVerified;
	private boolean m_voltageRampRateVerified;
	private boolean m_faultTimeVerified;

	//TODO merge
	private boolean m_controlEnabledJaguar;
	private boolean m_controlEnabledTalon;
	private JaguarControlMode m_controlModeJaguar;
	private TalonControlMode m_controlModeTalon;
	private byte m_deviceNumberJaguar;
	private int m_deviceNumberTalon;
	private ITable m_tableJaguar;
	private ITable m_tableTalon;
	private ITableListener m_table_listenerJaguar;
	private ITableListener m_table_listenerTalon;

	private void talonConstructor(int deviceNumber) {
		this.m_pidSource = PIDSourceType.kDisplacement;
		this.m_tableTalon = null;
		this.m_table_listenerTalon = null;
		this.m_deviceNumberTalon = deviceNumber;
		this.m_safetyHelper = new MotorSafetyHelper(this);
		this.m_controlEnabledTalon = true;
		this.m_profile = 0;
		this.m_setPoint = 0;
		this.m_codesPerRev = 0;
		this.m_numPotTurns = 0;
		this.m_feedbackDevice = FeedbackDevice.QuadEncoder;
		this.setProfile(this.m_profile);
		this.applyControlMode(TalonControlMode.PercentVbus);
		LiveWindow.addActuator("CANTalon", this.m_deviceNumberTalon, this);
	}
	private void jaguarConstructor(int deviceNumber) {
		this.m_neutralMode = NeutralMode.Jumper;
		this.m_encoderCodesPerRev = 0;
		this.m_potentiometerTurns = 0;
		this.m_limitMode = LimitMode.SwitchInputsOnly;
		this.m_forwardLimit = 0;
		this.m_reverseLimit = 0;
		this.m_maxOutputVoltage = 12;
		this.m_voltageRampRate = 0;
		this.m_faultTime = 0;
		this.m_controlModeVerified = true;
		this.m_speedRefVerified = true;
		this.m_posRefVerified = true;
		this.m_pVerified = true;
		this.m_iVerified = true;
		this.m_dVerified = true;
		this.m_neutralModeVerified = true;
		this.m_encoderCodesPerRevVerified = true;
		this.m_potentiometerTurnsVerified = true;
		this.m_forwardLimitVerified = true;
		this.m_reverseLimitVerified = true;
		this.m_limitModeVerified = true;
		this.m_maxOutputVoltageVerified = true;
		this.m_voltageRampRateVerified = true;
		this.m_faultTimeVerified = true;
		this.m_busVoltage = 0;
		this.m_outputVoltage = 0;
		this.m_outputCurrent = 0;
		this.m_temperature = 0;
		this.m_position = 0;
		this.m_speed = 0;
		this.m_limits = 0;
		this.m_firmwareVersion = 0;
		this.m_hardwareVersion = 0;
		this.m_receivedStatusMessages = new boolean[]{false, false, false};
		this.m_controlEnabledJaguar = true;
		this.m_tableJaguar = null;
		this.m_table_listenerJaguar = null;

		try {
			allocated.allocate(deviceNumber - 1);
		} catch (CheckedAllocationException e) {
			throw new AllocationException("GenericCANMotorController device " + e.getMessage() + "(increment index by one)");
		}

		this.m_deviceNumberJaguar = (byte)deviceNumber;
		this.m_controlModeJaguar = JaguarControlMode.PercentVbus;
		this.m_safetyHelper = new MotorSafetyHelper(this);
		byte[] data = new byte[8];
		this.requestMessage(-2147483136);
		this.requestMessage(520225088);

		for (int e = 0; e < 50; ++e) {
			Timer.delay(0.001);
			this.setupPeriodicStatus();
			this.updatePeriodicStatus();
			this.getMessage(512, 536870911, data);
			this.m_firmwareVersion = unpackINT32(data);
			if (this.m_receivedStatusMessages[0] && this.m_receivedStatusMessages[1] && this.m_receivedStatusMessages[2]) {
				break;
			}
		}

		if (this.m_receivedStatusMessages[0] && this.m_receivedStatusMessages[1] && this.m_receivedStatusMessages[2]) {
			try {
				this.getMessage(520225088, 536870911, data);
				this.m_hardwareVersion = data[0];
			} catch (CANMessageNotFoundException e) {
				this.m_hardwareVersion = 0;
			}

			if (this.m_firmwareVersion >= 3330 || this.m_firmwareVersion < 108) {
				if (this.m_firmwareVersion < 3330) {
					DriverStation.reportError("Jag " + this.m_deviceNumberJaguar + " firmware " + this.m_firmwareVersion + " is too old (must be at least version 108 of the FIRST approved firmware)", false);
				} else {
					DriverStation.reportError("Jag " + this.m_deviceNumberJaguar + " firmware " + this.m_firmwareVersion + " is not FIRST approved (must be at least version 108 of the FIRST approved firmware)", false);
				}

			}
		} else {
			this.free();
			throw new CANMessageNotFoundException();
		}
	}
	public GenericCANMotorController(RioCANID p, MotorControllerType m) throws CANMessageNotFoundException, CANIsUnsupportedException {
		if (m == MotorControllerType.Jaguar) {
			jaguarConstructor(p.getIDNumber());
		} else if (m == MotorControllerType.TalonSRX) {
			talonConstructor(p.getIDNumber());
			this.m_handle = CanTalonJNI.new_CanTalonSRX(p.getIDNumber());
		} else {
			throw new CANIsUnsupportedException(m);
		}
		port = p;
		type = m;
	}
	public GenericCANMotorController(RioCANID p, MotorControllerType m, int talonSRXCANcontrolPeriod_ms) throws CANMessageNotFoundException, CANIsUnsupportedException {
		if (m == MotorControllerType.Jaguar) {
			jaguarConstructor(p.getIDNumber());
		} else if (m == MotorControllerType.TalonSRX) {
			talonConstructor(p.getIDNumber());
			this.m_handle = CanTalonJNI.new_CanTalonSRX(p.getIDNumber(), talonSRXCANcontrolPeriod_ms);
		} else {
			throw new CANIsUnsupportedException(m);
		}
		port = p;
		type = m;
	}
	public GenericCANMotorController(RioCANID p, MotorControllerType m, int talonSRXCANcontrolPeriod_ms, int talonSRXCANEnablePeriod_ms) throws CANMessageNotFoundException, CANIsUnsupportedException {
		if (m == MotorControllerType.Jaguar) {
			jaguarConstructor(p.getIDNumber());
		} else if (m == MotorControllerType.TalonSRX) {
			talonConstructor(p.getIDNumber());
			this.m_handle = CanTalonJNI.new_CanTalonSRX(p.getIDNumber(), talonSRXCANcontrolPeriod_ms, talonSRXCANEnablePeriod_ms);
		} else {
			throw new CANIsUnsupportedException(m);
		}
		port = p;
		type = m;
	}

	public void free() {
		allocated.free(this.m_deviceNumberJaguar - 1);
		this.m_safetyHelper = null;
		int messageID;
		switch(this.m_controlModeJaguar.ordinal()) {
			case 1:
				messageID = this.m_deviceNumberJaguar | 33685824;
				break;
			case 2:
				messageID = this.m_deviceNumberJaguar | 33687040;
				break;
			case 3:
				messageID = this.m_deviceNumberJaguar | 33689088;
				break;
			case 4:
				messageID = this.m_deviceNumberJaguar | 33690048;
				break;
			case 5:
				messageID = this.m_deviceNumberJaguar | 33687936;
				break;
			default:
				return;
		}

		CANJNI.FRCNetworkCommunicationCANSessionMuxSendMessage(messageID, null, -1);
		this.configMaxOutputVoltage(12);
	}

	public double get() {
		return this.m_value;
	}

	public double getSetpoint() {
		return this.get();
	}

	public double getError() {
		return this.get() - this.getPosition();
	}

	public void set(double outputValue, byte syncGroup) {
		byte[] data = new byte[8];
		if (this.m_controlEnabledJaguar) {
			int messageID;
			byte dataSize;
			switch(this.m_controlModeJaguar.ordinal()) {
				case 1:
					messageID = 33685824;
					dataSize = packPercentage(data, this.isInverted?-outputValue:outputValue);
					break;
				case 2:
					messageID = 33687040;
					dataSize = packFXP16_16(data, this.isInverted?-outputValue:outputValue);
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
					dataSize = packFXP8_8(data, this.isInverted?-outputValue:outputValue);
					break;
				default:
					return;
			}

			if (syncGroup != 0) {
				data[dataSize++] = syncGroup;
			}

			this.sendMessage(messageID, data, dataSize, 20);
			if (this.m_safetyHelper != null) {
				this.m_safetyHelper.feed();
			}
		}

		this.m_value = outputValue;
		this.verify();
	}

	public void set(double value) {
		this.set(value, (byte)0);
	}

	public void setSetpoint(double value) {
		this.set(value);
	}

	public void reset() {
		this.set(this.m_value);
		this.disableControl();
	}

	public void setInverted(boolean isInverted) {
		this.isInverted = isInverted;
	}

	public boolean getInverted() {
		return this.isInverted;
	}

	public void verify() throws CANMessageNotFoundException {
		byte[] data = new byte[8];

		try {
			this.getMessage(33691136, 536870911, data);
			boolean e = data[0] != 0;
			if (e) {
				data[0] = 1;
				this.sendMessage(33691136, data, 1);
				this.m_controlModeVerified = false;
				this.m_speedRefVerified = false;
				this.m_posRefVerified = false;
				this.m_neutralModeVerified = false;
				this.m_encoderCodesPerRevVerified = false;
				this.m_potentiometerTurnsVerified = false;
				this.m_forwardLimitVerified = false;
				this.m_reverseLimitVerified = false;
				this.m_limitModeVerified = false;
				this.m_maxOutputVoltageVerified = false;
				this.m_faultTimeVerified = false;
				if (this.m_controlModeJaguar != JaguarControlMode.PercentVbus && this.m_controlModeJaguar != JaguarControlMode.Voltage) {
					this.m_pVerified = false;
					this.m_iVerified = false;
					this.m_dVerified = false;
				} else {
					this.m_voltageRampRateVerified = false;
				}

				this.m_receivedStatusMessages[0] = false;
				this.m_receivedStatusMessages[1] = false;
				this.m_receivedStatusMessages[2] = false;
				int[] e1 = new int[]{33686912, 33688960, 33686720, 33688768, 33689792, 33686784, 33688832, 33689856, 33686848, 33688896, 33689920, 33692736, 33692800, 33692864, 33692928, 33693056, 33693120, 33685696, 33687808, 33693184, 33692992};

				for (int message : e1) {
					this.getMessage(message, 536870911, data);
				}
			}
		} catch (CANMessageNotFoundException e) {
			this.requestMessage(33691136);
		}

		if (!this.m_controlModeVerified && this.m_controlEnabledJaguar) {
			try {
				this.getMessage(33691200, 536870911, data);
				if (this.m_controlModeJaguar == JaguarControlMode.values()[data[0]]) {
					this.m_controlModeVerified = true;
				} else {
					this.enableControl();
				}
			} catch (CANMessageNotFoundException e) {
				this.requestMessage(33691200);
			}
		}

		byte var28;
		if (!this.m_speedRefVerified) {
			try {
				this.getMessage(33686912, 536870911, data);
				var28 = data[0];
				if (this.m_speedReference == var28) {
					this.m_speedRefVerified = true;
				} else {
					this.setSpeedReference(this.m_speedReference);
				}
			} catch (CANMessageNotFoundException e) {
				this.requestMessage(33686912);
			}
		}

		if (!this.m_posRefVerified) {
			try {
				this.getMessage(33688960, 536870911, data);
				var28 = data[0];
				if (this.m_positionReference == var28) {
					this.m_posRefVerified = true;
				} else {
					this.setPositionReference(this.m_positionReference);
				}
			} catch (CANMessageNotFoundException e) {
				this.requestMessage(33688960);
			}
		}

		int var29;
		double var30;
		if (!this.m_pVerified) {
			var29 = 0;
			switch(this.m_controlModeJaguar.ordinal()) {
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
				this.getMessage(var29, 536870911, data);
				var30 = unpackFXP16_16(data);
				if (this.FXP16_EQ(this.m_p, var30)) {
					this.m_pVerified = true;
				} else {
					this.setP(this.m_p);
				}
			} catch (CANMessageNotFoundException e) {
				this.requestMessage(var29);
			}
		}

		if (!this.m_iVerified) {
			var29 = 0;
			switch(this.m_controlModeJaguar.ordinal()) {
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
				this.getMessage(var29, 536870911, data);
				var30 = unpackFXP16_16(data);
				if (this.FXP16_EQ(this.m_i, var30)) {
					this.m_iVerified = true;
				} else {
					this.setI(this.m_i);
				}
			} catch (CANMessageNotFoundException e) {
				this.requestMessage(var29);
			}
		}

		if (!this.m_dVerified) {
			var29 = 0;
			switch(this.m_controlModeJaguar.ordinal()) {
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
				this.getMessage(var29, 536870911, data);
				var30 = unpackFXP16_16(data);
				if (this.FXP16_EQ(this.m_d, var30)) {
					this.m_dVerified = true;
				} else {
					this.setD(this.m_d);
				}
			} catch (CANMessageNotFoundException e) {
				this.requestMessage(var29);
			}
		}

		if (!this.m_neutralModeVerified) {
			try {
				this.getMessage(33692864, 536870911, data);
				if (NeutralMode.valueOf(data[0]) == this.m_neutralMode) {
					this.m_neutralModeVerified = true;
				} else {
					this.configNeutralMode(this.m_neutralMode);
				}
			} catch (CANMessageNotFoundException e) {
				this.requestMessage(33692864);
			}
		}

		short var32;
		if (!this.m_encoderCodesPerRevVerified) {
			try {
				this.getMessage(33692736, 536870911, data);
				var32 = unpackINT16(data);
				if (var32 == this.m_encoderCodesPerRev) {
					this.m_encoderCodesPerRevVerified = true;
				} else {
					this.configEncoderCodesPerRev(this.m_encoderCodesPerRev);
				}
			} catch (CANMessageNotFoundException e) {
				this.requestMessage(33692736);
			}
		}

		if (!this.m_potentiometerTurnsVerified) {
			try {
				this.getMessage(33692800, 536870911, data);
				var32 = unpackINT16(data);
				if (var32 == this.m_potentiometerTurns) {
					this.m_potentiometerTurnsVerified = true;
				} else {
					this.configPotentiometerTurns(this.m_potentiometerTurns);
				}
			} catch (CANMessageNotFoundException e) {
				this.requestMessage(33692800);
			}
		}

		if (!this.m_limitModeVerified) {
			try {
				this.getMessage(33692928, 536870911, data);
				LimitMode var33 = LimitMode.valueOf(data[0]);
				if (var33 == this.m_limitMode) {
					this.m_limitModeVerified = true;
				} else {
					this.configLimitMode(this.m_limitMode);
				}
			} catch (CANMessageNotFoundException e) {
				this.requestMessage(33692928);
			}
		}

		double var34;
		if (!this.m_forwardLimitVerified) {
			try {
				this.getMessage(33692992, 536870911, data);
				var34 = unpackFXP16_16(data);
				if (this.FXP16_EQ(var34, this.m_forwardLimit)) {
					this.m_forwardLimitVerified = true;
				} else {
					this.configForwardLimit(this.m_forwardLimit);
				}
			} catch (CANMessageNotFoundException e) {
				this.requestMessage(33692992);
			}
		}

		if (!this.m_reverseLimitVerified) {
			try {
				this.getMessage(33693056, 536870911, data);
				var34 = unpackFXP16_16(data);
				if (this.FXP16_EQ(var34, this.m_reverseLimit)) {
					this.m_reverseLimitVerified = true;
				} else {
					this.configReverseLimit(this.m_reverseLimit);
				}
			} catch (CANMessageNotFoundException e) {
				this.requestMessage(33693056);
			}
		}

		if (!this.m_maxOutputVoltageVerified) {
			try {
				this.getMessage(33693120, 536870911, data);
				var34 = unpackFXP8_8(data);
				if (Math.abs(var34 - this.m_maxOutputVoltage) < 0.1) {
					this.m_maxOutputVoltageVerified = true;
				} else {
					this.configMaxOutputVoltage(this.m_maxOutputVoltage);
				}
			} catch (CANMessageNotFoundException e) {
				this.requestMessage(33693120);
			}
		}

		if (!this.m_voltageRampRateVerified) {
			if (this.m_controlModeJaguar == JaguarControlMode.PercentVbus) {
				try {
					this.getMessage(33685696, 536870911, data);
					var34 = unpackPercentage(data);
					if (this.FXP16_EQ(var34, this.m_voltageRampRate)) {
						this.m_voltageRampRateVerified = true;
					} else {
						this.setVoltageRampRate(this.m_voltageRampRate);
					}
				} catch (CANMessageNotFoundException e) {
					this.requestMessage(33685696);
				}
			}
		} else if (this.m_controlModeJaguar == JaguarControlMode.Voltage) {
			try {
				this.getMessage(33687808, 536870911, data);
				var34 = unpackFXP8_8(data);
				if (this.FXP8_EQ(var34, this.m_voltageRampRate)) {
					this.m_voltageRampRateVerified = true;
				} else {
					this.setVoltageRampRate(this.m_voltageRampRate);
				}
			} catch (CANMessageNotFoundException e) {
				this.requestMessage(33687808);
			}
		}

		if (!this.m_faultTimeVerified) {
			try {
				this.getMessage(33693184, 536870911, data);
				var32 = unpackINT16(data);
				if ((int)((double)this.m_faultTime * 1000) == var32) {
					this.m_faultTimeVerified = true;
				} else {
					this.configFaultTime(this.m_faultTime);
				}
			} catch (CANMessageNotFoundException e) {
				this.requestMessage(33693184);
			}
		}

		if (!this.m_receivedStatusMessages[0] || !this.m_receivedStatusMessages[1] || !this.m_receivedStatusMessages[2]) {
			this.setupPeriodicStatus();
			this.getTemperature();
			this.getPosition();
			this.getFaults();
		}

	}

	/** @deprecated */
	@Deprecated
	public void disable() {
		this.disableControl();
	}

	public void enable() {
		this.enableControl();
	}

	public void pidWrite(double output) {
		if (this.m_controlModeJaguar == JaguarControlMode.PercentVbus) {
			this.set(output);
		} else {
			throw new IllegalStateException("PID only supported in PercentVbus mode");
		}
	}

	private void setSpeedReference(int reference) {
		this.sendMessage(33686912, new byte[]{(byte)reference}, 1);
		this.m_speedReference = reference;
		this.m_speedRefVerified = false;
	}

	private void setPositionReference(int reference) {
		this.sendMessage(33688960, new byte[]{(byte)reference}, 1);
		this.m_positionReference = reference;
		this.m_posRefVerified = false;
	}

	public void setP(double p) {
		byte[] data = new byte[8];
		byte dataSize = packFXP16_16(data, p);
		switch(this.m_controlModeJaguar.ordinal()) {
			case 2:
				this.sendMessage(33686720, data, dataSize);
				break;
			case 3:
				this.sendMessage(33688768, data, dataSize);
				break;
			case 4:
				this.sendMessage(33689792, data, dataSize);
				break;
			default:
				throw new IllegalStateException("PID constants only apply in Speed, Position, and Current mode");
		}

		this.m_p = p;
		this.m_pVerified = false;
	}

	public void setI(double i) {
		byte[] data = new byte[8];
		byte dataSize = packFXP16_16(data, i);
		switch(this.m_controlModeJaguar.ordinal()) {
			case 2:
				this.sendMessage(33686784, data, dataSize);
				break;
			case 3:
				this.sendMessage(33688832, data, dataSize);
				break;
			case 4:
				this.sendMessage(33689856, data, dataSize);
				break;
			default:
				throw new IllegalStateException("PID constants only apply in Speed, Position, and Current mode");
		}

		this.m_i = i;
		this.m_iVerified = false;
	}

	public void setD(double d) {
		byte[] data = new byte[8];
		byte dataSize = packFXP16_16(data, d);
		switch(this.m_controlModeJaguar.ordinal()) {
			case 2:
				this.sendMessage(33686848, data, dataSize);
				break;
			case 3:
				this.sendMessage(33688896, data, dataSize);
				break;
			case 4:
				this.sendMessage(33689920, data, dataSize);
				break;
			default:
				throw new IllegalStateException("PID constants only apply in Speed, Position, and Current mode");
		}

		this.m_d = d;
		this.m_dVerified = false;
	}

	public void setPID(double p, double i, double d) {
		this.setP(p);
		this.setI(i);
		this.setD(d);
	}

	public double getP() {
		if (!this.m_controlModeJaguar.equals(JaguarControlMode.PercentVbus) && !this.m_controlModeJaguar.equals(JaguarControlMode.Voltage)) {
			return this.m_p;
		} else {
			throw new IllegalStateException("PID does not apply in Percent or Voltage control modes");
		}
	}

	public double getI() {
		if (!this.m_controlModeJaguar.equals(JaguarControlMode.PercentVbus) && !this.m_controlModeJaguar.equals(JaguarControlMode.Voltage)) {
			return this.m_i;
		} else {
			throw new IllegalStateException("PID does not apply in Percent or Voltage control modes");
		}
	}

	public double getD() {
		if (!this.m_controlModeJaguar.equals(JaguarControlMode.PercentVbus) && !this.m_controlModeJaguar.equals(JaguarControlMode.Voltage)) {
			return this.m_d;
		} else {
			throw new IllegalStateException("PID does not apply in Percent or Voltage control modes");
		}
	}

	public void enableControl(double encoderInitialPosition) {
		switch(this.m_controlModeJaguar.ordinal()) {
			case 1:
				this.sendMessage(33685760, new byte[0], 0);
				break;
			case 2:
				this.sendMessage(33686976, new byte[0], 0);
				break;
			case 3:
				byte[] data = new byte[8];
				byte dataSize = packFXP16_16(data, encoderInitialPosition);
				this.sendMessage(33689024, data, dataSize);
				break;
			case 4:
				this.sendMessage(33689984, new byte[0], 0);
				break;
			case 5:
				this.sendMessage(33687872, new byte[0], 0);
		}

		this.m_controlEnabledJaguar = true;
	}

	public void enableControl() {
		this.enableControl(0);
	}

	public boolean isEnabled() {
		return this.m_controlEnabledJaguar;
	}

	public void disableControl() {
		this.sendMessage(33685568, new byte[0], 0);
		this.sendMessage(33686592, new byte[0], 0);
		this.sendMessage(33688640, new byte[0], 0);
		this.sendMessage(33689664, new byte[0], 0);
		this.sendMessage(33687616, new byte[0], 0);
		this.sendMessage(33685824, new byte[0], 0, -1);
		this.sendMessage(33687040, new byte[0], 0, -1);
		this.sendMessage(33689088, new byte[0], 0, -1);
		this.sendMessage(33690048, new byte[0], 0, -1);
		this.sendMessage(33687936, new byte[0], 0, -1);
		this.m_controlEnabledJaguar = false;
	}

	public void setPercentMode() {
		this.changeControlMode(JaguarControlMode.PercentVbus);
		this.setPositionReference(255);
		this.setSpeedReference(255);
	}
	public void setPercentMode_Encoder(int codesPerRev, boolean isQuadEncoder) {
		this.changeControlMode(JaguarControlMode.PercentVbus);
		this.configEncoderCodesPerRev(codesPerRev);
		if (isQuadEncoder) {
			this.setPositionReference(0);
			this.setSpeedReference(3);
		} else {
			this.setPositionReference(255);
			this.setSpeedReference(0);
		}
	}
	public void setPercentMode_Potentiometer() {
		this.changeControlMode(JaguarControlMode.PercentVbus);
		this.setPositionReference(1);
		this.setSpeedReference(255);
		this.configPotentiometerTurns(1);
	}

	public void setCurrentMode(double p, double i, double d) {
		this.changeControlMode(JaguarControlMode.Current);
		this.setPositionReference(255);
		this.setSpeedReference(255);
		this.setPID(p, i, d);
	}
	public void setCurrentMode_Encoder(double p, double i, double d, int codesPerRev, boolean isQuadEncoder) {
		this.changeControlMode(JaguarControlMode.Current);
		this.configEncoderCodesPerRev(codesPerRev);
		this.setPID(p, i, d);
		if (isQuadEncoder) {
			this.setPositionReference(0);
			this.setSpeedReference(3);
		} else {
			this.setPositionReference(255);
			this.setSpeedReference(255);
		}
	}
	public void setCurrentMode_Potentiometer(double p, double i, double d) {
		this.changeControlMode(JaguarControlMode.Current);
		this.setPositionReference(1);
		this.setSpeedReference(255);
		this.configPotentiometerTurns(1);
		this.setPID(p, i, d);
	}

	public void setSpeedMode(double p, double i, double d, int codesPerRev, boolean isQuadEncoder) {
		this.changeControlMode(JaguarControlMode.Speed);
		this.configEncoderCodesPerRev(codesPerRev);
		this.setPID(p, i, d);
		if (isQuadEncoder) {
			this.setPositionReference(0);
			this.setSpeedReference(3);
		} else {
			this.setPositionReference(255);
			this.setSpeedReference(0);
		}
	}

	public void setPositionMode_QuadEncoder(double p, double i, double d, int codesPerRev) {
		this.changeControlMode(JaguarControlMode.Position);
		this.setPositionReference(0);
		this.configEncoderCodesPerRev(codesPerRev);
		this.setPID(p, i, d);
	}
	public void setPositionMode_Potentiometer(double p, double i, double d) {
		this.changeControlMode(JaguarControlMode.Position);
		this.setPositionReference(1);
		this.configPotentiometerTurns(1);
		this.setPID(p, i, d);
	}

	public void setVoltageMode() {
		this.changeControlMode(JaguarControlMode.Voltage);
		this.setPositionReference(255);
		this.setSpeedReference(255);
	}
	public void setVoltageMode_Encoder(int codesPerRev, boolean isQuadEncoder) {
		this.changeControlMode(JaguarControlMode.Voltage);
		this.configEncoderCodesPerRev(codesPerRev);
		if (isQuadEncoder) {
			this.setPositionReference(0);
			this.setSpeedReference(3);
		} else {
			this.setPositionReference(255);
			this.setSpeedReference(0);
		}
	}
	public void setVoltageMode_Potentiometer() {
		this.changeControlMode(JaguarControlMode.Voltage);
		this.setPositionReference(1);
		this.setSpeedReference(255);
		this.configPotentiometerTurns(1);
	}

	private void changeControlMode(JaguarControlMode controlMode) {
		this.disableControl();
		this.m_controlModeJaguar = controlMode;
		this.m_controlModeVerified = false;
	}

	public JaguarControlMode getControlMode() {
		return this.m_controlModeJaguar;
	}

	public void setControlMode(int mode) {
		this.changeControlMode(JaguarControlMode.values()[mode]);
	}

	public double getBusVoltage() {
		this.updatePeriodicStatus();
		return this.m_busVoltage;
	}

	public double getOutputVoltage() {
		this.updatePeriodicStatus();
		return this.m_outputVoltage;
	}

	public double getOutputCurrent() {
		this.updatePeriodicStatus();
		return this.m_outputCurrent;
	}

	public double getTemperature() {
		this.updatePeriodicStatus();
		return this.m_temperature;
	}

	public double getPosition() {
		this.updatePeriodicStatus();
		return this.m_position;
	}

	public double getSpeed() {
		this.updatePeriodicStatus();
		return this.m_speed;
	}

	public boolean getForwardLimitOK() {
		this.updatePeriodicStatus();
		return (this.m_limits & 1) != 0;
	}

	public boolean getReverseLimitOK() {
		this.updatePeriodicStatus();
		return (this.m_limits & 2) != 0;
	}

	public void getFaults() {
		this.updatePeriodicStatus();
	}

	public void setVoltageRampRate(double rampRate) {
		byte[] data = new byte[8];
		byte dataSize;
		int message;
		switch(this.m_controlModeJaguar.ordinal()) {
			case 1:
				dataSize = packPercentage(data, rampRate / (this.m_maxOutputVoltage * 1000));
				message = 33685696;
				break;
			case 5:
				dataSize = packFXP8_8(data, rampRate / 1000);
				message = 33687808;
				break;
			default:
				throw new IllegalStateException("Voltage ramp rate only applies in Percentage and Voltage modes");
		}

		this.sendMessage(message, data, dataSize);
	}

	public int getFirmwareVersion() {
		return this.m_firmwareVersion;
	}

	public byte getHardwareVersion() {
		return this.m_hardwareVersion;
	}

	public void configNeutralMode(NeutralMode mode) {
		this.sendMessage(33692864, new byte[]{mode.value}, 1);
		this.m_neutralMode = mode;
		this.m_neutralModeVerified = false;
	}

	public void configEncoderCodesPerRev(int codesPerRev) {
		byte[] data = new byte[8];
		byte dataSize = packINT16(data, (short)codesPerRev);
		this.sendMessage(33692736, data, dataSize);
		this.m_encoderCodesPerRev = (short)codesPerRev;
		this.m_encoderCodesPerRevVerified = false;
	}

	public void configPotentiometerTurns(int turns) {
		byte[] data = new byte[8];
		byte dataSize = packINT16(data, (short)turns);
		this.sendMessage(33692800, data, dataSize);
		this.m_potentiometerTurns = (short)turns;
		this.m_potentiometerTurnsVerified = false;
	}

	public void configSoftPositionLimits(double forwardLimitPosition, double reverseLimitPosition) {
		this.configLimitMode(LimitMode.SoftPositionLimits);
		this.configForwardLimit(forwardLimitPosition);
		this.configReverseLimit(reverseLimitPosition);
	}

	public void disableSoftPositionLimits() {
		this.configLimitMode(LimitMode.SwitchInputsOnly);
	}

	public void configLimitMode(LimitMode mode) {
		this.sendMessage(33692928, new byte[]{mode.value}, 1);
	}

	public void configForwardLimit(double forwardLimitPosition) {
		byte[] data = new byte[8];
		byte dataSize = packFXP16_16(data, forwardLimitPosition);
		int dataSize1 = dataSize + 1;
		data[dataSize] = 1;
		this.sendMessage(33692992, data, dataSize1);
		this.m_forwardLimit = forwardLimitPosition;
		this.m_forwardLimitVerified = false;
	}

	public void configReverseLimit(double reverseLimitPosition) {
		byte[] data = new byte[8];
		byte dataSize = packFXP16_16(data, reverseLimitPosition);
		int dataSize1 = dataSize + 1;
		data[dataSize] = 1;
		this.sendMessage(33693056, data, dataSize1);
		this.m_reverseLimit = reverseLimitPosition;
		this.m_reverseLimitVerified = false;
	}

	public void configMaxOutputVoltage(double voltage) {
		byte[] data = new byte[8];
		byte dataSize = packFXP8_8(data, voltage);
		this.sendMessage(33693120, data, dataSize);
		this.m_maxOutputVoltage = voltage;
		this.m_maxOutputVoltageVerified = false;
	}

	public void configFaultTime(float faultTime) {
		byte[] data = new byte[8];
		if (faultTime < 0.5) {
			faultTime = (float) 0.5;
		} else if (faultTime > 3) {
			faultTime = 3;
		}

		byte dataSize = packINT16(data, (short)((int)((double)faultTime * 1000)));
		this.sendMessage(33693184, data, dataSize);
		this.m_faultTime = faultTime;
		this.m_faultTimeVerified = false;
	}

	static void sendMessageHelper(int messageID, byte[] data, int dataSize, int period) throws CANMessageNotFoundException {
		int[] kTrustedMessages = new int[]{33685760, 33685824, 33686976, 33687040, 33687872, 33687936, 33689024, 33689088, 33689984, 33690048};

		for (int kTrustedMessage : kTrustedMessages) {
			if ((536870848 & messageID) == kTrustedMessage) {
				if (dataSize > 6) {
					throw new RuntimeException("CAN message has too much data.");
				}

				ByteBuffer i = ByteBuffer.allocateDirect(dataSize + 2);
				i.put(0, (byte) 0);
				i.put(1, (byte) 0);

				for (byte j = 0; j < dataSize; ++j) {
					i.put(j + 2, data[j]);
				}

				CANJNI.FRCNetworkCommunicationCANSessionMuxSendMessage(messageID, i, period);
				return;
			}
		}

		ByteBuffer var8;
		if (data != null) {
			var8 = ByteBuffer.allocateDirect(dataSize);

			for (byte var9 = 0; var9 < dataSize; ++var9) {
				var8.put(var9, data[var9]);
			}
		} else {
			var8 = null;
		}

		CANJNI.FRCNetworkCommunicationCANSessionMuxSendMessage(messageID, var8, period);
	}

	public void sendMessage(int messageID, byte[] data, int dataSize, int period) {
		sendMessageHelper(messageID | this.m_deviceNumberJaguar, data, dataSize, period);
	}

	public void sendMessage(int messageID, byte[] data, int dataSize) {
		this.sendMessage(messageID, data, dataSize, 0);
	}

	public void requestMessage(int messageID, int period) {
		sendMessageHelper(messageID | this.m_deviceNumberJaguar, null, 0, period);
	}

	public void requestMessage(int messageID) {
		this.requestMessage(messageID, 0);
	}

	public void getMessage(int messageID, int messageMask, byte[] data) throws CANMessageNotFoundException {
		messageID |= this.m_deviceNumberJaguar;
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
		byte[] kMessage0Data = new byte[]{(byte)3, (byte)4, (byte)1, (byte)2, (byte)5, (byte)6, (byte)7, (byte)8};
		byte[] kMessage1Data = new byte[]{(byte)9, (byte)10, (byte)11, (byte)12, (byte)13, (byte)14, (byte)15, (byte)16};
		byte[] kMessage2Data = new byte[]{(byte)18, (byte)19, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0, (byte)0};
		byte dataSize = packINT16(data, (short)20);
		this.sendMessage(33691648, data, dataSize);
		this.sendMessage(33691712, data, dataSize);
		this.sendMessage(33691776, data, dataSize);
		byte dataSize1 = 8;
		this.sendMessage(33691904, kMessage0Data, dataSize1);
		this.sendMessage(33691968, kMessage1Data, dataSize1);
		this.sendMessage(33692032, kMessage2Data, dataSize1);
	}

	public void updatePeriodicStatus() throws CANMessageNotFoundException {
		byte[] data = new byte[8];

		this.getMessage(33692160, 536870911, data);
		this.m_busVoltage = unpackFXP8_8(new byte[]{data[0], data[1]});
		this.m_outputVoltage = unpackPercentage(new byte[]{data[2], data[3]}) * this.m_busVoltage;
		this.m_outputCurrent = unpackFXP8_8(new byte[]{data[4], data[5]});
		this.m_temperature = unpackFXP8_8(new byte[]{data[6], data[7]});
		this.m_receivedStatusMessages[0] = true;

		this.getMessage(33692224, 536870911, data);
		this.m_position = unpackFXP16_16(new byte[]{data[0], data[1], data[2], data[3]});
		this.m_speed = unpackFXP16_16(new byte[]{data[4], data[5], data[6], data[7]});
		this.m_receivedStatusMessages[1] = true;

		this.getMessage(33692288, 536870911, data);
		this.m_limits = data[0];
		this.m_receivedStatusMessages[2] = true;

	}

	private static void swap16(int x, byte[] buffer) {
		buffer[0] = (byte)(x & 255);
		buffer[1] = (byte)(x >> 8 & 255);
	}

	private static void swap32(int x, byte[] buffer) {
		buffer[0] = (byte)(x & 255);
		buffer[1] = (byte)(x >> 8 & 255);
		buffer[2] = (byte)(x >> 16 & 255);
		buffer[3] = (byte)(x >> 24 & 255);
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
		return (byte)2;
	}

	private static byte packFXP8_8(byte[] buffer, double value) {
		short intValue = (short)((int)(value * 256));
		swap16(intValue, buffer);
		return (byte)2;
	}

	private static byte packFXP16_16(byte[] buffer, double value) {
		int intValue = (int)(value * 65536);
		swap32(intValue, buffer);
		return (byte)4;
	}

	private static byte packINT16(byte[] buffer, short value) {
		swap16(value, buffer);
		return (byte)2;
	}

	private static short unpack16(byte[] buffer, int offset) {
		return (short)(buffer[offset] & 255 | (short)(buffer[offset + 1] << 8) & '\uff00');
	}

	private static int unpack32(byte[] buffer, int offset) {
		return buffer[offset] & 255 | buffer[offset + 1] << 8 & '\uff00' | buffer[offset + 2] << 16 & 16711680 | buffer[offset + 3] << 24 & -16777216;
	}

	private static double unpackPercentage(byte[] buffer) {
		return (double)unpack16(buffer, 0) / 32767;
	}

	private static double unpackFXP8_8(byte[] buffer) {
		return (double)unpack16(buffer, 0) / 256;
	}

	private static double unpackFXP16_16(byte[] buffer) {
		return (double)unpack32(buffer, 0) / 65536;
	}

	private static short unpackINT16(byte[] buffer) {
		return unpack16(buffer, 0);
	}

	private static int unpackINT32(byte[] buffer) {
		return unpack32(buffer, 0);
	}

	public boolean FXP8_EQ(double a, double b) {
		return (int)(a * 256) == (int)(b * 256);
	}

	public boolean FXP16_EQ(double a, double b) {
		return (int)(a * 65536) == (int)(b * 65536);
	}

	public void setExpiration(double timeout) {
		this.m_safetyHelper.setExpiration(timeout);
	}

	public double getExpiration() {
		return this.m_safetyHelper.getExpiration();
	}

	public boolean isAlive() {
		return this.m_safetyHelper.isAlive();
	}

	public boolean isSafetyEnabled() {
		return this.m_safetyHelper.isSafetyEnabled();
	}

	public void setSafetyEnabled(boolean enabled) {
		this.m_safetyHelper.setSafetyEnabled(enabled);
	}

	public String getDescription() {
		return "GenericCANMotorController ID " + this.m_deviceNumberJaguar;
	}

	public int getDeviceID() {
		return this.m_deviceNumberJaguar;
	}

	/** @deprecated */
	@Deprecated
	public void stopMotor() {
		this.disableControl();
	}

	public void initTable(ITable subtable) {
		this.m_tableJaguar = subtable;
		this.updateTable();
	}

	public ITable getTable() {
		return this.m_tableJaguar;
	}

	public void startLiveWindowMode() {
		switch (type) {
			case Jaguar:
				this.set(0);
				this.m_table_listenerJaguar = this.createTableListener();
				this.m_tableJaguar.addTableListener(this.m_table_listenerJaguar, true);
				break;
			case TalonSRX:
				this.set_Talon(0);
				this.m_table_listenerTalon = this.createTableListener();
				this.m_tableTalon.addTableListener(this.m_table_listenerTalon, true);
				break;
		}
	}

	public void stopLiveWindowMode() {
		switch (type) {
			case Jaguar:
				this.set(0);
				this.m_tableJaguar.removeTableListener(this.m_table_listenerJaguar);
				break;
			case TalonSRX:
				this.set_Talon(0);
				this.m_tableTalon.removeTableListener(this.m_table_listenerTalon);
				break;
		}
	}

	public void pidWrite_Talon(double output) {
		if (this.getControlMode_Talon() == TalonControlMode.PercentVbus) {
			this.set_Talon(output);
		} else {
			throw new IllegalStateException("PID only supported in PercentVbus mode");
		}
	}

	public void setPIDSourceType(PIDSourceType pidSource) {
		this.m_pidSource = pidSource;
	}

	public PIDSourceType getPIDSourceType() {
		return this.m_pidSource;
	}

	public double pidGet() {
		return this.getPosition_Talon();
	}

	public void delete() {
		this.disable_Talon();
		if (this.m_handle != 0L) {
			CanTalonJNI.delete_CanTalonSRX(this.m_handle);
			this.m_handle = 0L;
		}

	}

	public void set_Talon(double outputValue) {
		this.m_safetyHelper.feed();
		if (this.m_controlEnabledTalon) {
			this.m_setPoint = outputValue;
			switch(this.m_controlModeTalon.ordinal()) {
				case 1:
					CanTalonJNI.Set(this.m_handle, this.isInverted?-outputValue:outputValue);
					break;
				case 2:
					CanTalonJNI.SetDemand(this.m_handle, (int)outputValue);
					break;
				case 3:
					int volts = (int)((this.isInverted?-outputValue:outputValue) * 256);
					CanTalonJNI.SetDemand(this.m_handle, volts);
					break;
				case 4:
					CanTalonJNI.SetDemand(this.m_handle, this.ScaleVelocityToNativeUnits(this.m_feedbackDevice, this.isInverted?-outputValue:outputValue));
					break;
				case 5:
					CanTalonJNI.SetDemand(this.m_handle, this.ScaleRotationsToNativeUnits(this.m_feedbackDevice, outputValue));
					break;
				case 6:
					double milliamperes = (this.isInverted?-outputValue:outputValue) * 1000;
					CanTalonJNI.SetDemand(this.m_handle, (int)milliamperes);
					break;
				case 7:
					CanTalonJNI.SetDemand(this.m_handle, (int)outputValue);
			}

			CanTalonJNI.SetModeSelect(this.m_handle, this.m_controlModeTalon.value);
		}

	}

	public void setInverted_Talon(boolean isInverted) {
		this.isInverted = isInverted;
	}

	public boolean getInverted_Talon() {
		return this.isInverted;
	}

	public void reset_Talon() {
		this.disable_Talon();
		this.clearIAccum();
	}

	public boolean isEnabled_Talon() {
		return this.isControlEnabled();
	}

	public double getError_Talon() {
		return (double)this.getClosedLoopError();
	}

	public void setSetpoint_Talon(double setpoint) {
		this.set_Talon(setpoint);
	}

	public void reverseSensor(boolean flip) {
		CanTalonJNI.SetRevFeedbackSensor(this.m_handle, flip?1:0);
	}

	public void reverseOutput(boolean flip) {
		CanTalonJNI.SetRevMotDuringCloseLoopEn(this.m_handle, flip?1:0);
	}

	public double get_Talon() {
		switch(this.m_controlModeTalon.ordinal()) {
			case 1:
			case 2:
			default:
				return (double)CanTalonJNI.GetAppliedThrottle(this.m_handle) / 1023;
			case 3:
				return this.getOutputVoltage_Talon();
			case 4:
				return this.ScaleNativeUnitsToRpm(this.m_feedbackDevice, (long)CanTalonJNI.GetSensorVelocity(this.m_handle));
			case 5:
				return this.ScaleNativeUnitsToRotations(this.m_feedbackDevice, CanTalonJNI.GetSensorPosition(this.m_handle));
			case 6:
				return this.getOutputCurrent_Talon();
		}
	}

	public int getEncPosition() {
		return CanTalonJNI.GetEncPosition(this.m_handle);
	}

	public void setEncPosition(int newPosition) {
		this.setParameter(CanTalonJNI.param_t.eEncPosition, (double)newPosition);
	}

	public int getEncVelocity() {
		return CanTalonJNI.GetEncVel(this.m_handle);
	}

	public int getPulseWidthPosition() {
		return CanTalonJNI.GetPulseWidthPosition(this.m_handle);
	}

	public void setPulseWidthPosition(int newPosition) {
		this.setParameter(CanTalonJNI.param_t.ePwdPosition, (double)newPosition);
	}

	public int getPulseWidthVelocity() {
		return CanTalonJNI.GetPulseWidthVelocity(this.m_handle);
	}

	public int getPulseWidthRiseToFallUs() {
		return CanTalonJNI.GetPulseWidthRiseToFallUs(this.m_handle);
	}

	public int getPulseWidthRiseToRiseUs() {
		return CanTalonJNI.GetPulseWidthRiseToRiseUs(this.m_handle);
	}

	public FeedbackDeviceStatus isSensorPresent(FeedbackDevice feedbackDevice) {
		FeedbackDeviceStatus toReturn = FeedbackDeviceStatus.FeedbackStatusUnknown;
		switch(feedbackDevice.ordinal()) {
			case 6:
			case 7:
			case 8:
				if (CanTalonJNI.IsPulseWidthSensorPresent(this.m_handle) == 0) {
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
		return CanTalonJNI.GetEncIndexRiseEvents(this.m_handle);
	}

	public int getPinStateQuadA() {
		return CanTalonJNI.GetQuadApin(this.m_handle);
	}

	public int getPinStateQuadB() {
		return CanTalonJNI.GetQuadBpin(this.m_handle);
	}

	public int getPinStateQuadIdx() {
		return CanTalonJNI.GetQuadIdxpin(this.m_handle);
	}

	public void setAnalogPosition(int newPosition) {
		this.setParameter(CanTalonJNI.param_t.eAinPosition, (double)newPosition);
	}

	public int getAnalogInPosition() {
		return CanTalonJNI.GetAnalogInWithOv(this.m_handle);
	}

	public int getAnalogInRaw() {
		return this.getAnalogInPosition() & 1023;
	}

	public int getAnalogInVelocity() {
		return CanTalonJNI.GetAnalogInVel(this.m_handle);
	}

	public int getClosedLoopError() {
		return CanTalonJNI.GetCloseLoopErr(this.m_handle);
	}

	public void setAllowableClosedLoopErr(int allowableCloseLoopError) {
		if (this.m_profile == 0) {
			this.setParameter(CanTalonJNI.param_t.eProfileParamSlot0_AllowableClosedLoopErr, (double)allowableCloseLoopError);
		} else {
			this.setParameter(CanTalonJNI.param_t.eProfileParamSlot1_AllowableClosedLoopErr, (double)allowableCloseLoopError);
		}

	}

	public boolean isFwdLimitSwitchClosed() {
		return CanTalonJNI.GetLimitSwitchClosedFor(this.m_handle) == 0;
	}

	public boolean isRevLimitSwitchClosed() {
		return CanTalonJNI.GetLimitSwitchClosedRev(this.m_handle) == 0;
	}

	public boolean getBrakeEnableDuringNeutral() {
		return CanTalonJNI.GetBrakeIsEnabled(this.m_handle) != 0;
	}

	public void configEncoderCodesPerRev_Talon(int codesPerRev) {
		this.m_codesPerRev = codesPerRev;
		this.setParameter(CanTalonJNI.param_t.eNumberEncoderCPR, (double)this.m_codesPerRev);
	}

	public void configPotentiometerTurns_Talon(int turns) {
		this.m_numPotTurns = turns;
		this.setParameter(CanTalonJNI.param_t.eNumberPotTurns, (double)this.m_numPotTurns);
	}

	public double getTemperature_Talon() {
		return CanTalonJNI.GetTemp(this.m_handle);
	}

	public double getOutputCurrent_Talon() {
		return CanTalonJNI.GetCurrent(this.m_handle);
	}

	public double getOutputVoltage_Talon() {
		return this.getBusVoltage_Talon() * (double)CanTalonJNI.GetAppliedThrottle(this.m_handle) / 1023;
	}

	public double getBusVoltage_Talon() {
		return CanTalonJNI.GetBatteryV(this.m_handle);
	}

	public double getPosition_Talon() {
		return this.ScaleNativeUnitsToRotations(this.m_feedbackDevice, CanTalonJNI.GetSensorPosition(this.m_handle));
	}

	public void setPosition(double pos) {
		int nativePos = this.ScaleRotationsToNativeUnits(this.m_feedbackDevice, pos);
		CanTalonJNI.SetSensorPosition(this.m_handle, nativePos);
	}

	public double getSpeed_Talon() {
		return this.ScaleNativeUnitsToRpm(this.m_feedbackDevice, (long)CanTalonJNI.GetSensorVelocity(this.m_handle));
	}

	public TalonControlMode getControlMode_Talon() {
		return this.m_controlModeTalon;
	}

	public void setControlMode_Talon(int mode) {
		this.changeControlMode(TalonControlMode.valueOf(mode));
	}

	private void applyControlMode(TalonControlMode controlMode) {
		this.m_controlModeTalon = controlMode;
		if (controlMode == TalonControlMode.Disabled) {
			this.m_controlEnabledTalon = false;
		}

		CanTalonJNI.SetModeSelect(this.m_handle, TalonControlMode.Disabled.value);
		UsageReporting.report(52, this.m_deviceNumberTalon + 1, controlMode.value);
	}

	public void changeControlMode(TalonControlMode controlMode) {
		if (this.m_controlModeTalon != controlMode) {
			this.applyControlMode(controlMode);
		}

	}

	public void setFeedbackDevice(FeedbackDevice device) {
		this.m_feedbackDevice = device;
		CanTalonJNI.SetFeedbackDeviceSelect(this.m_handle, device.value);
	}

	public void setStatusFrameRateMs(StatusFrameRate stateFrame, int periodMs) {
		CanTalonJNI.SetStatusFrameRate(this.m_handle, stateFrame.value, periodMs);
	}

	public void enableControl_Talon() {
		this.changeControlMode(this.m_controlModeTalon);
		this.m_controlEnabledTalon = true;
	}

	public void enable_Talon() {
		this.enableControl_Talon();
	}

	public void disableControl_Talon() {
		CanTalonJNI.SetModeSelect(this.m_handle, TalonControlMode.Disabled.value);
		this.m_controlEnabledTalon = false;
	}

	public boolean isControlEnabled() {
		return this.m_controlEnabledTalon;
	}

	public double getP_Talon() {
		if (this.m_profile == 0) {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot0_P.value);
		} else {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot1_P.value);
		}

		Timer.delay(kDelayForSolicitedSignals);
		return CanTalonJNI.GetPgain(this.m_handle, this.m_profile);
	}

	public double getI_Talon() {
		if (this.m_profile == 0) {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot0_I.value);
		} else {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot1_I.value);
		}

		Timer.delay(kDelayForSolicitedSignals);
		return CanTalonJNI.GetIgain(this.m_handle, this.m_profile);
	}

	public double getD_Talon() {
		if (this.m_profile == 0) {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot0_D.value);
		} else {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot1_D.value);
		}

		Timer.delay(kDelayForSolicitedSignals);
		return CanTalonJNI.GetDgain(this.m_handle, this.m_profile);
	}

	public double getF() {
		if (this.m_profile == 0) {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot0_F.value);
		} else {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot1_F.value);
		}

		Timer.delay(kDelayForSolicitedSignals);
		return CanTalonJNI.GetFgain(this.m_handle, this.m_profile);
	}

	public double getIZone() {
		if (this.m_profile == 0) {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot0_IZone.value);
		} else {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot1_IZone.value);
		}

		Timer.delay(kDelayForSolicitedSignals);
		return (double)CanTalonJNI.GetIzone(this.m_handle, this.m_profile);
	}

	public double getCloseLoopRampRate() {
		if (this.m_profile == 0) {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot0_CloseLoopRampRate.value);
		} else {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot1_CloseLoopRampRate.value);
		}

		Timer.delay(kDelayForSolicitedSignals);
		double throttlePerMs = (double)CanTalonJNI.GetCloseLoopRampRate(this.m_handle, this.m_profile);
		return throttlePerMs / 1023 * 12000;
	}

	public long GetFirmwareVersion() {
		CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eFirmVers.value);
		Timer.delay(kDelayForSolicitedSignals);
		return (long)CanTalonJNI.GetParamResponseInt32(this.m_handle, CanTalonJNI.param_t.eFirmVers.value);
	}

	public long GetIaccum() {
		CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.ePidIaccum.value);
		Timer.delay(kDelayForSolicitedSignals);
		return (long)CanTalonJNI.GetParamResponseInt32(this.m_handle, CanTalonJNI.param_t.ePidIaccum.value);
	}

	public void setP_Talon(double p) {
		CanTalonJNI.SetPgain(this.m_handle, this.m_profile, p);
	}

	public void setI_Talon(double i) {
		CanTalonJNI.SetIgain(this.m_handle, this.m_profile, i);
	}

	public void setD_Talon(double d) {
		CanTalonJNI.SetDgain(this.m_handle, this.m_profile, d);
	}

	public void setF(double f) {
		CanTalonJNI.SetFgain(this.m_handle, this.m_profile, f);
	}

	public void setIZone(int izone) {
		CanTalonJNI.SetIzone(this.m_handle, this.m_profile, izone);
	}

	public void setCloseLoopRampRate(double rampRate) {
		int rate = (int)(rampRate * 1023 / 12 / 1000);
		CanTalonJNI.SetCloseLoopRampRate(this.m_handle, this.m_profile, rate);
	}

	public void setVoltageRampRate_Talon(double rampRate) {
		int rate = (int)(rampRate * 1023 / 12 / 100);
		CanTalonJNI.SetRampThrottle(this.m_handle, rate);
	}

	public void setVoltageCompensationRampRate(double rampRate) {
		CanTalonJNI.SetVoltageCompensationRate(this.m_handle, rampRate / 1000);
	}

	public void ClearIaccum() {
		CanTalonJNI.SetParam(this.m_handle, CanTalonJNI.param_t.ePidIaccum.value, 0);
	}

	public void setPID_Talon(double p, double i, double d, double f, int izone, double closeLoopRampRate, int profile) {
		if (profile != 0 && profile != 1) {
			throw new IllegalArgumentException("Talon PID profile must be 0 or 1.");
		} else {
			this.m_profile = profile;
			this.setProfile(profile);
			this.setP_Talon(p);
			this.setI_Talon(i);
			this.setD_Talon(d);
			this.setF(f);
			this.setIZone(izone);
			this.setCloseLoopRampRate(closeLoopRampRate);
		}
	}

	public void setPID_Talon(double p, double i, double d) {
		this.setPID_Talon(p, i, d, 0, 0, 0, this.m_profile);
	}

	public double getSetpoint_Talon() {
		return this.m_setPoint;
	}

	public void setProfile(int profile) {
		if (profile != 0 && profile != 1) {
			throw new IllegalArgumentException("Talon PID profile must be 0 or 1.");
		} else {
			this.m_profile = profile;
			CanTalonJNI.SetProfileSlotSelect(this.m_handle, this.m_profile);
		}
	}

	/** @deprecated */
	@Deprecated
	public void stopMotor_Talon() {
		this.disableControl_Talon();
	}

	public void disable_Talon() {
		this.disableControl_Talon();
	}

	public void clearIAccum() {
		CanTalonJNI.SetParam(this.m_handle, CanTalonJNI.param_t.ePidIaccum.value, 0);
	}

	public void setForwardSoftLimit(double forwardLimit) {
		int nativeLimitPos = this.ScaleRotationsToNativeUnits(this.m_feedbackDevice, forwardLimit);
		CanTalonJNI.SetForwardSoftLimit(this.m_handle, nativeLimitPos);
	}

	public int getForwardSoftLimit() {
		return CanTalonJNI.GetForwardSoftLimit(this.m_handle);
	}

	public void enableForwardSoftLimit(boolean enable) {
		CanTalonJNI.SetForwardSoftEnable(this.m_handle, enable?1:0);
	}

	public boolean isForwardSoftLimitEnabled() {
		return CanTalonJNI.GetForwardSoftEnable(this.m_handle) != 0;
	}

	public void setReverseSoftLimit(double reverseLimit) {
		int nativeLimitPos = this.ScaleRotationsToNativeUnits(this.m_feedbackDevice, reverseLimit);
		CanTalonJNI.SetReverseSoftLimit(this.m_handle, nativeLimitPos);
	}

	public int getReverseSoftLimit() {
		return CanTalonJNI.GetReverseSoftLimit(this.m_handle);
	}

	public void enableReverseSoftLimit(boolean enable) {
		CanTalonJNI.SetReverseSoftEnable(this.m_handle, enable?1:0);
	}

	public boolean isReverseSoftLimitEnabled() {
		return CanTalonJNI.GetReverseSoftEnable(this.m_handle) != 0;
	}

	public void configMaxOutputVoltage_Talon(double voltage) {
		this.configPeakOutputVoltage(voltage, -voltage);
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

		this.setParameter(CanTalonJNI.param_t.ePeakPosOutput, 1023 * forwardVoltage / 12);
		this.setParameter(CanTalonJNI.param_t.ePeakNegOutput, 1023 * reverseVoltage / 12);
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

		this.setParameter(CanTalonJNI.param_t.eNominalPosOutput, 1023 * forwardVoltage / 12);
		this.setParameter(CanTalonJNI.param_t.eNominalNegOutput, 1023 * reverseVoltage / 12);
	}

	public void setParameter(CanTalonJNI.param_t paramEnum, double value) {
		CanTalonJNI.SetParam(this.m_handle, paramEnum.value, value);
	}

	public double getParameter(CanTalonJNI.param_t paramEnum) {
		CanTalonJNI.RequestParam(this.m_handle, paramEnum.value);
		Timer.delay(kDelayForSolicitedSignals);
		return CanTalonJNI.GetParamResponse(this.m_handle, paramEnum.value);
	}

	public void clearStickyFaults() {
		CanTalonJNI.ClearStickyFaults(this.m_handle);
	}

	public void enableLimitSwitch(boolean forward, boolean reverse) {
		int mask = 4 + (forward?1:0) * 2 + (reverse?1:0);
		CanTalonJNI.SetOverrideLimitSwitchEn(this.m_handle, mask);
	}

	public void ConfigFwdLimitSwitchNormallyOpen(boolean normallyOpen) {
		CanTalonJNI.SetParam(this.m_handle, CanTalonJNI.param_t.eOnBoot_LimitSwitch_Forward_NormallyClosed.value, normallyOpen?0:1);
	}

	public void ConfigRevLimitSwitchNormallyOpen(boolean normallyOpen) {
		CanTalonJNI.SetParam(this.m_handle, CanTalonJNI.param_t.eOnBoot_LimitSwitch_Reverse_NormallyClosed.value, normallyOpen?0:1);
	}

	public void enableBrakeMode(boolean brake) {
		CanTalonJNI.SetOverrideBrakeType(this.m_handle, brake?2:1);
	}

	public int getFaultOverTemp() {
		return CanTalonJNI.GetFault_OverTemp(this.m_handle);
	}

	public int getFaultUnderVoltage() {
		return CanTalonJNI.GetFault_UnderVoltage(this.m_handle);
	}

	public int getFaultForLim() {
		return CanTalonJNI.GetFault_ForLim(this.m_handle);
	}

	public int getFaultRevLim() {
		return CanTalonJNI.GetFault_RevLim(this.m_handle);
	}

	public int getFaultHardwareFailure() {
		return CanTalonJNI.GetFault_HardwareFailure(this.m_handle);
	}

	public int getFaultForSoftLim() {
		return CanTalonJNI.GetFault_ForSoftLim(this.m_handle);
	}

	public int getFaultRevSoftLim() {
		return CanTalonJNI.GetFault_RevSoftLim(this.m_handle);
	}

	public int getStickyFaultOverTemp() {
		return CanTalonJNI.GetStckyFault_OverTemp(this.m_handle);
	}

	public int getStickyFaultUnderVoltage() {
		return CanTalonJNI.GetStckyFault_UnderVoltage(this.m_handle);
	}

	public int getStickyFaultForLim() {
		return CanTalonJNI.GetStckyFault_ForLim(this.m_handle);
	}

	public int getStickyFaultRevLim() {
		return CanTalonJNI.GetStckyFault_RevLim(this.m_handle);
	}

	public int getStickyFaultForSoftLim() {
		return CanTalonJNI.GetStckyFault_ForSoftLim(this.m_handle);
	}

	public int getStickyFaultRevSoftLim() {
		return CanTalonJNI.GetStckyFault_RevSoftLim(this.m_handle);
	}

	private double GetNativeUnitsPerRotationScalar(FeedbackDevice devToLookup) {
		double toReturn = 0;
		boolean scalingAvail = false;
		switch(devToLookup.ordinal()) {
			case 1:
				switch(this.m_feedbackDevice.ordinal()) {
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

				if (!scalingAvail && 0 != this.m_codesPerRev) {
					toReturn = (double)(4 * this.m_codesPerRev);
					scalingAvail = true;
				}
				break;
			case 2:
			case 3:
				if (0 != this.m_numPotTurns) {
					toReturn = 1024 / (double)this.m_numPotTurns;
					scalingAvail = true;
				}
				break;
			case 4:
			case 5:
				if (0 != this.m_codesPerRev) {
					toReturn = (double)(this.m_codesPerRev);
					scalingAvail = true;
				}
				break;
			case 6:
			case 7:
			case 8:
				toReturn = 4096;
				scalingAvail = true;
		}

		return !scalingAvail?0:toReturn;
	}

	private int ScaleRotationsToNativeUnits(FeedbackDevice devToLookup, double fullRotations) {
		int toReturn = (int)fullRotations;
		double scalar = this.GetNativeUnitsPerRotationScalar(devToLookup);
		if (scalar > 0) {
			toReturn = (int)(fullRotations * scalar);
		}

		return toReturn;
	}

	private int ScaleVelocityToNativeUnits(FeedbackDevice devToLookup, double rpm) {
		int toReturn = (int)rpm;
		double scalar = this.GetNativeUnitsPerRotationScalar(devToLookup);
		if (scalar > 0) {
			toReturn = (int)(rpm / 600 * scalar);
		}

		return toReturn;
	}

	private double ScaleNativeUnitsToRotations(FeedbackDevice devToLookup, int nativePos) {
		double toReturn = (double)nativePos;
		double scalar = this.GetNativeUnitsPerRotationScalar(devToLookup);
		if (scalar > 0) {
			toReturn = (double)nativePos / scalar;
		}

		return toReturn;
	}

	private double ScaleNativeUnitsToRpm(FeedbackDevice devToLookup, long nativeVel) {
		double toReturn = (double)nativeVel;
		double scalar = this.GetNativeUnitsPerRotationScalar(devToLookup);
		if (scalar > 0) {
			toReturn = (double)nativeVel / (scalar / 600);
		}

		return toReturn;
	}

	public void enableZeroSensorPositionOnIndex(boolean enable, boolean risingEdge) {
		if (enable) {
			this.setParameter(CanTalonJNI.param_t.eQuadIdxPolarity, risingEdge?1:0);
			this.setParameter(CanTalonJNI.param_t.eClearPositionOnIdx, 1);
		} else {
			this.setParameter(CanTalonJNI.param_t.eClearPositionOnIdx, 0);
			this.setParameter(CanTalonJNI.param_t.eQuadIdxPolarity, risingEdge?1:0);
		}

	}

	public void changeMotionControlFramePeriod(int periodMs) {
		CanTalonJNI.ChangeMotionControlFramePeriod(this.m_handle, periodMs);
	}

	public void clearMotionProfileTrajectories() {
		CanTalonJNI.ClearMotionProfileTrajectories(this.m_handle);
	}

	public int getMotionProfileTopLevelBufferCount() {
		return CanTalonJNI.GetMotionProfileTopLevelBufferCount(this.m_handle);
	}

	public boolean pushMotionProfileTrajectory(TrajectoryPoint trajPt) {
		if (this.isMotionProfileTopLevelBufferFull()) {
			return false;
		} else {
			int targPos = this.ScaleRotationsToNativeUnits(this.m_feedbackDevice, trajPt.position);
			int targVel = this.ScaleVelocityToNativeUnits(this.m_feedbackDevice, trajPt.velocity);
			int profileSlotSelect = trajPt.profileSlotSelect > 0?1:0;
			int timeDurMs = trajPt.timeDurMs;
			if (timeDurMs > 255) {
				timeDurMs = 255;
			}

			if (timeDurMs < 0) {
				timeDurMs = 0;
			}

			CanTalonJNI.PushMotionProfileTrajectory(this.m_handle, targPos, targVel, profileSlotSelect, timeDurMs, trajPt.velocityOnly?1:0, trajPt.isLastPoint?1:0, trajPt.zeroPos?1:0);
			return true;
		}
	}

	public boolean isMotionProfileTopLevelBufferFull() {
		return CanTalonJNI.IsMotionProfileTopLevelBufferFull(this.m_handle);
	}

	public void processMotionProfileBuffer() {
		CanTalonJNI.ProcessMotionProfileBuffer(this.m_handle);
	}

	public void getMotionProfileStatus() {
		CanTalonJNI.GetMotionProfileStatus(this.m_handle, this, motionProfileStatus);
	}

	public void setMotionProfileStatusFromJNI(int flags, int profileSlotSelect, int targPos, int targVel, int topBufferRem, int topBufferCnt, int btmBufferCnt, int outputEnable) {
		motionProfileStatus = new MotionProfileStatus(topBufferRem, topBufferCnt, btmBufferCnt, (flags & 2) > 0, (flags & 4) > 0, (flags & 1) > 0, SetValueMotionProfile.valueOf(outputEnable));
		motionProfileStatus.activePoint.isLastPoint = (flags & 8) > 0;
		motionProfileStatus.activePoint.velocityOnly = (flags & 16) > 0;
		motionProfileStatus.activePoint.position = this.ScaleNativeUnitsToRotations(this.m_feedbackDevice, targPos);
		motionProfileStatus.activePoint.velocity = this.ScaleNativeUnitsToRpm(this.m_feedbackDevice, (long)targVel);
		motionProfileStatus.activePoint.profileSlotSelect = profileSlotSelect;
		motionProfileStatus.activePoint.zeroPos = false;
		motionProfileStatus.activePoint.timeDurMs = 0;
	}

	public void clearMotionProfileHasUnderrun() {
		this.setParameter(CanTalonJNI.param_t.eMotionProfileHasUnderrunErr, 0);
	}

	private static class MotionProfileStatus {
		int topBufferRem;
		int topBufferCnt;
		int btmBufferCnt;
		boolean hasUnderrun;
		boolean isUnderrun;
		boolean activePointValid;
		final TrajectoryPoint activePoint = new TrajectoryPoint();
		SetValueMotionProfile outputEnable;

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
