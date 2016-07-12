package org.firstinspires.frc.framework.granulation.to_delete;

import edu.wpi.first.wpilibj.CANSpeedController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Resource;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.can.CANJNI;
import edu.wpi.first.wpilibj.can.CANMessageNotFoundException;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;
import edu.wpi.first.wpilibj.util.AllocationException;
import edu.wpi.first.wpilibj.util.CheckedAllocationException;
import org.firstinspires.frc.framework.hardware.MotorController_Individual;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class GenericCANMotorControllerPart1 implements MotorSafety, PIDOutput, CANSpeedController {
	private MotorSafetyHelper m_safetyHelper;
	private static final Resource allocated = new Resource(63);
	private boolean isInverted = false;
	private byte m_deviceNumber;
	private double m_value = 0;
	private JaguarControlMode m_controlMode;
	private int m_speedReference = 255;
	private int m_positionReference = 255;
	private double m_p = 0;
	private double m_i = 0;
	private double m_d = 0;
	private NeutralMode m_neutralMode;
	private short m_encoderCodesPerRev;
	private short m_potentiometerTurns;
	private final LimitMode m_limitMode;
	private double m_forwardLimit;
	private double m_reverseLimit;
	private double m_maxOutputVoltage;
	private final double m_voltageRampRate;
	private float m_faultTime;
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
	private double m_busVoltage;
	private double m_outputVoltage;
	private double m_outputCurrent;
	private double m_temperature;
	private double m_position;
	private double m_speed;
	private byte m_limits;
	private int m_firmwareVersion;
	private byte m_hardwareVersion;
	private boolean m_receivedStatusMessage0;
	private boolean m_receivedStatusMessage1;
	private boolean m_receivedStatusMessage2;
	private boolean m_controlEnabled;
	private ITable m_table;
	private ITableListener m_table_listener;

	public GenericCANMotorControllerPart1(int deviceNumber) throws CANMessageNotFoundException {
		this.m_neutralMode = NeutralMode.Jumper;
		this.m_encoderCodesPerRev = 0;
		this.m_potentiometerTurns = 0;
		this.m_limitMode = LimitMode.SwitchInputsOnly;
		this.m_forwardLimit = 0;
		this.m_reverseLimit = 0;
		this.m_maxOutputVoltage = 12.0D;
		this.m_voltageRampRate = 0;
		this.m_faultTime = 0.0F;
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
		this.m_receivedStatusMessage0 = false;
		this.m_receivedStatusMessage1 = false;
		this.m_receivedStatusMessage2 = false;
		this.m_controlEnabled = true;
		this.m_table = null;
		this.m_table_listener = null;

		try {
			allocated.allocate(deviceNumber - 1);
		} catch (CheckedAllocationException var8) {
			throw new AllocationException("GenericCANMotorControllerPart1 device " + var8.getMessage() + "(increment index by one)");
		}

		this.m_deviceNumber = (byte)deviceNumber;
		this.m_controlMode = JaguarControlMode.PercentVbus;
		this.m_safetyHelper = new MotorSafetyHelper(this);
		byte[] data = new byte[8];
		this.requestMessage(-2147483136);
		this.requestMessage(520225088);

		for(int e = 0; e < 50; ++e) {
			Timer.delay(0.001D);
			this.setupPeriodicStatus();
			this.updatePeriodicStatus();
			this.getMessage(512, 536870911, data);
			this.m_firmwareVersion = unpackINT32(data);

			if(this.m_receivedStatusMessage0 && this.m_receivedStatusMessage1 && this.m_receivedStatusMessage2) {
				break;
			}
		}

		if(this.m_receivedStatusMessage0 && this.m_receivedStatusMessage1 && this.m_receivedStatusMessage2) {
			try {
				this.getMessage(520225088, 536870911, data);
				this.m_hardwareVersion = data[0];
			} catch (CANMessageNotFoundException var6) {
				this.m_hardwareVersion = 0;
			}

			if(this.m_firmwareVersion >= 3330 || this.m_firmwareVersion < 108) {
				if(this.m_firmwareVersion < 3330) {
					DriverStation.reportError("Jag " + this.m_deviceNumber + " firmware " + this.m_firmwareVersion + " is too old (must be at least version 108 of the FIRST approved firmware)", false);
				} else {
					DriverStation.reportError("Jag" + this.m_deviceNumber + " firmware " + this.m_firmwareVersion + " is not FIRST approved (must be at least version 108 of the FIRST approved firmware)", false);
				}

			}
		} else {
			this.free();
			throw new CANMessageNotFoundException();
		}
	}

	@SuppressWarnings("WeakerAccess")
	public void free() {
		allocated.free(this.m_deviceNumber - 1);
		this.m_safetyHelper = null;
		int messageID;
		switch(this.m_controlMode.ordinal()) {
			case 1:
				messageID = this.m_deviceNumber | 33685824;
				break;
			case 2:
				messageID = this.m_deviceNumber | 33687040;
				break;
			case 3:
				messageID = this.m_deviceNumber | 33689088;
				break;
			case 4:
				messageID = this.m_deviceNumber | 33690048;
				break;
			case 5:
				messageID = this.m_deviceNumber | 33687936;
				break;
			default:
				return;
		}

		CANJNI.FRCNetworkCommunicationCANSessionMuxSendMessage(messageID, null, -1);
		this.configMaxOutputVoltage(12.0D);
	}

	int getDeviceNumber() {
		return this.m_deviceNumber;
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
		if(this.m_controlEnabled) {
			int messageID;
			byte dataSize;
			switch(this.m_controlMode.ordinal()) {
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

			if(syncGroup != 0) {
				data[dataSize++] = syncGroup;
			}

			this.sendMessage(messageID, data, dataSize, 20);
			if(this.m_safetyHelper != null) {
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

	@SuppressWarnings("WeakerAccess")
	protected void verify() throws CANMessageNotFoundException {
		byte[] data = new byte[8];

		try {
			this.getMessage(33691136, 536870911, data);
			boolean e = data[0] != 0;
			if(e) {
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
				if(this.m_controlMode != JaguarControlMode.PercentVbus && this.m_controlMode != JaguarControlMode.Voltage) {
					this.m_pVerified = false;
					this.m_iVerified = false;
					this.m_dVerified = false;
				} else {
					this.m_voltageRampRateVerified = false;
				}

				this.m_receivedStatusMessage0 = false;
				this.m_receivedStatusMessage1 = false;
				this.m_receivedStatusMessage2 = false;
				int[] e1 = new int[]{33686912, 33688960, 33686720, 33688768, 33689792, 33686784, 33688832, 33689856, 33686848, 33688896, 33689920, 33692736, 33692800, 33692864, 33692928, 33693056, 33693120, 33685696, 33687808, 33693184, 33692992};

				for (int message : e1) {
					this.getMessage(message, 536870911, data);
				}
			}
		} catch (CANMessageNotFoundException var26) {
			this.requestMessage(33691136);
		}

		if(!this.m_controlModeVerified && this.m_controlEnabled) {
			try {
				this.getMessage(33691200, 536870911, data);
				JaguarControlMode var27 = JaguarControlMode.values()[data[0]];
				if(this.m_controlMode == var27) {
					this.m_controlModeVerified = true;
				} else {
					this.enableControl();
				}
			} catch (CANMessageNotFoundException var24) {
				this.requestMessage(33691200);
			}
		}

		byte var28;
		if(!this.m_speedRefVerified) {
			try {
				this.getMessage(33686912, 536870911, data);
				var28 = data[0];
				if(this.m_speedReference == var28) {
					this.m_speedRefVerified = true;
				} else {
					this.setSpeedReference(this.m_speedReference);
				}
			} catch (CANMessageNotFoundException var23) {
				this.requestMessage(33686912);
			}
		}

		if(!this.m_posRefVerified) {
			try {
				this.getMessage(33688960, 536870911, data);
				var28 = data[0];
				if(this.m_positionReference == var28) {
					this.m_posRefVerified = true;
				} else {
					this.setPositionReference(this.m_positionReference);
				}
			} catch (CANMessageNotFoundException var22) {
				this.requestMessage(33688960);
			}
		}

		int var29;
		double var30;
		if(!this.m_pVerified) {
			var29 = 0;
			switch(this.m_controlMode.ordinal()) {
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
				if(this.FXP16_EQ(this.m_p, var30)) {
					this.m_pVerified = true;
				} else {
					this.setP(this.m_p);
				}
			} catch (CANMessageNotFoundException var21) {
				this.requestMessage(var29);
			}
		}

		if(!this.m_iVerified) {
			var29 = 0;
			switch(this.m_controlMode.ordinal()) {
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
				if(this.FXP16_EQ(this.m_i, var30)) {
					this.m_iVerified = true;
				} else {
					this.setI(this.m_i);
				}
			} catch (CANMessageNotFoundException var20) {
				this.requestMessage(var29);
			}
		}

		if(!this.m_dVerified) {
			var29 = 0;
			switch(this.m_controlMode.ordinal()) {
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
				if(this.FXP16_EQ(this.m_d, var30)) {
					this.m_dVerified = true;
				} else {
					this.setD(this.m_d);
				}
			} catch (CANMessageNotFoundException var19) {
				this.requestMessage(var29);
			}
		}

		if(!this.m_neutralModeVerified) {
			try {
				this.getMessage(33692864, 536870911, data);
				NeutralMode var31 = NeutralMode.valueOf(data[0]);
				if(var31 == this.m_neutralMode) {
					this.m_neutralModeVerified = true;
				} else {
					this.configNeutralMode(this.m_neutralMode);
				}
			} catch (CANMessageNotFoundException var18) {
				this.requestMessage(33692864);
			}
		}

		short var32;
		if(!this.m_encoderCodesPerRevVerified) {
			try {
				this.getMessage(33692736, 536870911, data);
				var32 = unpackINT16(data);
				if(var32 == this.m_encoderCodesPerRev) {
					this.m_encoderCodesPerRevVerified = true;
				} else {
					this.configEncoderCodesPerRev(this.m_encoderCodesPerRev);
				}
			} catch (CANMessageNotFoundException var17) {
				this.requestMessage(33692736);
			}
		}

		if(!this.m_potentiometerTurnsVerified) {
			try {
				this.getMessage(33692800, 536870911, data);
				var32 = unpackINT16(data);
				if(var32 == this.m_potentiometerTurns) {
					this.m_potentiometerTurnsVerified = true;
				} else {
					this.configPotentiometerTurns(this.m_potentiometerTurns);
				}
			} catch (CANMessageNotFoundException var16) {
				this.requestMessage(33692800);
			}
		}

		if(!this.m_limitModeVerified) {
			try {
				this.getMessage(33692928, 536870911, data);
				LimitMode var33 = LimitMode.valueOf(data[0]);
				if(var33 == this.m_limitMode) {
					this.m_limitModeVerified = true;
				} else {
					this.configLimitMode(this.m_limitMode);
				}
			} catch (CANMessageNotFoundException var15) {
				this.requestMessage(33692928);
			}
		}

		double var34;
		if(!this.m_forwardLimitVerified) {
			try {
				this.getMessage(33692992, 536870911, data);
				var34 = unpackFXP16_16(data);
				if(this.FXP16_EQ(var34, this.m_forwardLimit)) {
					this.m_forwardLimitVerified = true;
				} else {
					this.configForwardLimit(this.m_forwardLimit);
				}
			} catch (CANMessageNotFoundException var14) {
				this.requestMessage(33692992);
			}
		}

		if(!this.m_reverseLimitVerified) {
			try {
				this.getMessage(33693056, 536870911, data);
				var34 = unpackFXP16_16(data);
				if(this.FXP16_EQ(var34, this.m_reverseLimit)) {
					this.m_reverseLimitVerified = true;
				} else {
					this.configReverseLimit(this.m_reverseLimit);
				}
			} catch (CANMessageNotFoundException var13) {
				this.requestMessage(33693056);
			}
		}

		if(!this.m_maxOutputVoltageVerified) {
			try {
				this.getMessage(33693120, 536870911, data);
				var34 = unpackFXP8_8(data);
				if(Math.abs(var34 - this.m_maxOutputVoltage) < 0.1D) {
					this.m_maxOutputVoltageVerified = true;
				} else {
					this.configMaxOutputVoltage(this.m_maxOutputVoltage);
				}
			} catch (CANMessageNotFoundException var12) {
				this.requestMessage(33693120);
			}
		}

		if(!this.m_voltageRampRateVerified) {
			if(this.m_controlMode == JaguarControlMode.PercentVbus) {
				try {
					this.getMessage(33685696, 536870911, data);
					var34 = unpackPercentage(data);
					if(this.FXP16_EQ(var34, this.m_voltageRampRate)) {
						this.m_voltageRampRateVerified = true;
					} else {
						this.setVoltageRampRate(this.m_voltageRampRate);
					}
				} catch (CANMessageNotFoundException var11) {
					this.requestMessage(33685696);
				}
			}
		} else if(this.m_controlMode == JaguarControlMode.Voltage) {
			try {
				this.getMessage(33687808, 536870911, data);
				var34 = unpackFXP8_8(data);
				if(this.FXP8_EQ(var34, this.m_voltageRampRate)) {
					this.m_voltageRampRateVerified = true;
				} else {
					this.setVoltageRampRate(this.m_voltageRampRate);
				}
			} catch (CANMessageNotFoundException var10) {
				this.requestMessage(33687808);
			}
		}

		if(!this.m_faultTimeVerified) {
			try {
				this.getMessage(33693184, 536870911, data);
				var32 = unpackINT16(data);
				if((int)((double)this.m_faultTime * 1000) == var32) {
					this.m_faultTimeVerified = true;
				} else {
					this.configFaultTime(this.m_faultTime);
				}
			} catch (CANMessageNotFoundException var9) {
				this.requestMessage(33693184);
			}
		}

		if(!this.m_receivedStatusMessage0 || !this.m_receivedStatusMessage1 || !this.m_receivedStatusMessage2) {
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
		if(this.m_controlMode == JaguarControlMode.PercentVbus) {
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
		switch(this.m_controlMode.ordinal()) {
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
		switch(this.m_controlMode.ordinal()) {
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
		switch(this.m_controlMode.ordinal()) {
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
		if(!this.m_controlMode.equals(JaguarControlMode.PercentVbus) && !this.m_controlMode.equals(JaguarControlMode.Voltage)) {
			return this.m_p;
		} else {
			throw new IllegalStateException("PID does not apply in Percent or Voltage control modes");
		}
	}

	public double getI() {
		if(!this.m_controlMode.equals(JaguarControlMode.PercentVbus) && !this.m_controlMode.equals(JaguarControlMode.Voltage)) {
			return this.m_i;
		} else {
			throw new IllegalStateException("PID does not apply in Percent or Voltage control modes");
		}
	}

	public double getD() {
		if(!this.m_controlMode.equals(JaguarControlMode.PercentVbus) && !this.m_controlMode.equals(JaguarControlMode.Voltage)) {
			return this.m_d;
		} else {
			throw new IllegalStateException("PID does not apply in Percent or Voltage control modes");
		}
	}

	@SuppressWarnings("WeakerAccess")
	public void enableControl(@SuppressWarnings("SameParameterValue") double encoderInitialPosition) {
		switch(this.m_controlMode.ordinal()) {
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

		this.m_controlEnabled = true;
	}

	@SuppressWarnings("WeakerAccess")
	public void enableControl() {
		this.enableControl(0);
	}

	public boolean isEnabled() {
		return this.m_controlEnabled;
	}

	@SuppressWarnings("WeakerAccess")
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
		this.m_controlEnabled = false;
	}

	public void setPercentMode() {
		this.changeControlMode(JaguarControlMode.PercentVbus);
		this.setPositionReference(255);
		this.setSpeedReference(255);
	}

	public void setPercentMode(EncoderTag tag, int codesPerRev) {
		this.changeControlMode(JaguarControlMode.PercentVbus);
		this.setPositionReference(255);
		this.setSpeedReference(0);
		this.configEncoderCodesPerRev(codesPerRev);
	}

	public void setPercentMode(QuadEncoderTag tag, int codesPerRev) {
		this.changeControlMode(JaguarControlMode.PercentVbus);
		this.setPositionReference(0);
		this.setSpeedReference(3);
		this.configEncoderCodesPerRev(codesPerRev);
	}

	public void setPercentMode(PotentiometerTag tag) {
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

	public void setCurrentMode(EncoderTag tag, int codesPerRev, double p, double i, double d) {
		this.changeControlMode(JaguarControlMode.Current);
		this.setPositionReference(255);
		this.setSpeedReference(255);
		this.configEncoderCodesPerRev(codesPerRev);
		this.setPID(p, i, d);
	}

	public void setCurrentMode(QuadEncoderTag tag, int codesPerRev, double p, double i, double d) {
		this.changeControlMode(JaguarControlMode.Current);
		this.setPositionReference(0);
		this.setSpeedReference(3);
		this.configEncoderCodesPerRev(codesPerRev);
		this.setPID(p, i, d);
	}

	public void setCurrentMode(PotentiometerTag tag, double p, double i, double d) {
		this.changeControlMode(JaguarControlMode.Current);
		this.setPositionReference(1);
		this.setSpeedReference(255);
		this.configPotentiometerTurns(1);
		this.setPID(p, i, d);
	}

	public void setSpeedMode(EncoderTag tag, int codesPerRev, double p, double i, double d) {
		this.changeControlMode(JaguarControlMode.Speed);
		this.setPositionReference(255);
		this.setSpeedReference(0);
		this.configEncoderCodesPerRev(codesPerRev);
		this.setPID(p, i, d);
	}

	public void setSpeedMode(QuadEncoderTag tag, int codesPerRev, double p, double i, double d) {
		this.changeControlMode(JaguarControlMode.Speed);
		this.setPositionReference(0);
		this.setSpeedReference(3);
		this.configEncoderCodesPerRev(codesPerRev);
		this.setPID(p, i, d);
	}

	public void setPositionMode(QuadEncoderTag tag, int codesPerRev, double p, double i, double d) {
		this.changeControlMode(JaguarControlMode.Position);
		this.setPositionReference(0);
		this.configEncoderCodesPerRev(codesPerRev);
		this.setPID(p, i, d);
	}

	public void setPositionMode(PotentiometerTag tag, double p, double i, double d) {
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

	public void setVoltageMode(EncoderTag tag, int codesPerRev) {
		this.changeControlMode(JaguarControlMode.Voltage);
		this.setPositionReference(255);
		this.setSpeedReference(0);
		this.configEncoderCodesPerRev(codesPerRev);
	}

	public void setVoltageMode(QuadEncoderTag tag, int codesPerRev) {
		this.changeControlMode(JaguarControlMode.Voltage);
		this.setPositionReference(0);
		this.setSpeedReference(3);
		this.configEncoderCodesPerRev(codesPerRev);
	}

	public void setVoltageMode(PotentiometerTag tag) {
		this.changeControlMode(JaguarControlMode.Voltage);
		this.setPositionReference(1);
		this.setSpeedReference(255);
		this.configPotentiometerTurns(1);
	}

	private void changeControlMode(JaguarControlMode controlMode) {
		this.disableControl();
		this.m_controlMode = controlMode;
		this.m_controlModeVerified = false;
	}

	public JaguarControlMode getControlMode() {
		return this.m_controlMode;
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

	@SuppressWarnings("WeakerAccess")
	public void getFaults() {
		this.updatePeriodicStatus();
	}

	public void setVoltageRampRate(double rampRate) {
		byte[] data = new byte[8];
		byte dataSize;
		int message;
		switch(this.m_controlMode.ordinal()) {
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

	@SuppressWarnings("WeakerAccess")
	public void configNeutralMode(NeutralMode mode) {
		this.sendMessage(33692864, new byte[]{mode.value}, 1);
		this.m_neutralMode = mode;
		this.m_neutralModeVerified = false;
	}

	@SuppressWarnings("WeakerAccess")
	public void configEncoderCodesPerRev(int codesPerRev) {
		byte[] data = new byte[8];
		byte dataSize = packINT16(data, (short)codesPerRev);
		this.sendMessage(33692736, data, dataSize);
		this.m_encoderCodesPerRev = (short)codesPerRev;
		this.m_encoderCodesPerRevVerified = false;
	}

	@SuppressWarnings("WeakerAccess")
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

	@SuppressWarnings("WeakerAccess")
	public void configLimitMode(LimitMode mode) {
		this.sendMessage(33692928, new byte[]{mode.value}, 1);
	}

	@SuppressWarnings("WeakerAccess")
	public void configForwardLimit(double forwardLimitPosition) {
		byte[] data = new byte[8];
		byte dataSize = packFXP16_16(data, forwardLimitPosition);
		int dataSize1 = dataSize + 1;
		data[dataSize] = 1;
		this.sendMessage(33692992, data, dataSize1);
		this.m_forwardLimit = forwardLimitPosition;
		this.m_forwardLimitVerified = false;
	}

	@SuppressWarnings("WeakerAccess")
	public void configReverseLimit(double reverseLimitPosition) {
		byte[] data = new byte[8];
		byte dataSize = packFXP16_16(data, reverseLimitPosition);
		int dataSize1 = dataSize + 1;
		data[dataSize] = 1;
		this.sendMessage(33693056, data, dataSize1);
		this.m_reverseLimit = reverseLimitPosition;
		this.m_reverseLimitVerified = false;
	}

	@SuppressWarnings("WeakerAccess")
	public void configMaxOutputVoltage(double voltage) {
		byte[] data = new byte[8];
		byte dataSize = packFXP8_8(data, voltage);
		this.sendMessage(33693120, data, dataSize);
		this.m_maxOutputVoltage = voltage;
		this.m_maxOutputVoltageVerified = false;
	}

	@SuppressWarnings("WeakerAccess")
	public void configFaultTime(float faultTime) {
		byte[] data = new byte[8];
		if(faultTime < 0.5F) {
			faultTime = 0.5F;
		} else if(faultTime > 3.0F) {
			faultTime = 3.0F;
		}

		byte dataSize = packINT16(data, (short)((int)((double)faultTime * 1000)));
		this.sendMessage(33693184, data, dataSize);
		this.m_faultTime = faultTime;
		this.m_faultTimeVerified = false;
	}

	@SuppressWarnings("WeakerAccess")
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
		if(data != null) {
			var8 = ByteBuffer.allocateDirect(dataSize);

			for(byte var9 = 0; var9 < dataSize; ++var9) {
				var8.put(var9, data[var9]);
			}
		} else {
			var8 = null;
		}

		CANJNI.FRCNetworkCommunicationCANSessionMuxSendMessage(messageID, var8, period);
	}

	@SuppressWarnings("WeakerAccess")
	protected void sendMessage(int messageID, byte[] data, int dataSize, int period) {
		sendMessageHelper(messageID | this.m_deviceNumber, data, dataSize, period);
	}

	@SuppressWarnings("WeakerAccess")
	protected void sendMessage(int messageID, byte[] data, int dataSize) {
		this.sendMessage(messageID, data, dataSize, 0);
	}

	@SuppressWarnings("WeakerAccess")
	protected void requestMessage(int messageID, int period) {
		sendMessageHelper(messageID | this.m_deviceNumber, null, 0, period);
	}

	@SuppressWarnings("WeakerAccess")
	protected void requestMessage(int messageID) {
		this.requestMessage(messageID, 0);
	}

	@SuppressWarnings("WeakerAccess")
	protected void getMessage(int messageID, int messageMask, byte[] data) throws CANMessageNotFoundException {
		messageID |= this.m_deviceNumber;
		messageID &= 536870911;
		ByteBuffer targetedMessageID = ByteBuffer.allocateDirect(4);
		targetedMessageID.order(ByteOrder.LITTLE_ENDIAN);
		targetedMessageID.asIntBuffer().put(0, messageID);
		ByteBuffer timeStamp = ByteBuffer.allocateDirect(4);
		ByteBuffer dataBuffer = CANJNI.FRCNetworkCommunicationCANSessionMuxReceiveMessage(targetedMessageID.asIntBuffer(), messageMask, timeStamp);
		if(data != null) {
			for(int i = 0; i < dataBuffer.capacity(); ++i) {
				data[i] = dataBuffer.get(i);
			}
		}

	}

	@SuppressWarnings("WeakerAccess")
	protected void setupPeriodicStatus() {
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

	@SuppressWarnings("WeakerAccess")
	protected void updatePeriodicStatus() throws CANMessageNotFoundException {
		byte[] data = new byte[8];

		this.getMessage(33692160, 536870911, data);
		this.m_busVoltage = unpackFXP8_8(new byte[]{data[0], data[1]});
		this.m_outputVoltage = unpackPercentage(new byte[]{data[2], data[3]}) * this.m_busVoltage;
		this.m_outputCurrent = unpackFXP8_8(new byte[]{data[4], data[5]});
		this.m_temperature = unpackFXP8_8(new byte[]{data[6], data[7]});
		this.m_receivedStatusMessage0 = true;

		this.getMessage(33692224, 536870911, data);
		this.m_position = unpackFXP16_16(new byte[]{data[0], data[1], data[2], data[3]});
		this.m_speed = unpackFXP16_16(new byte[]{data[4], data[5], data[6], data[7]});
		this.m_receivedStatusMessage1 = true;

		this.getMessage(33692288, 536870911, data);
		this.m_limits = data[0];
		this.m_receivedStatusMessage2 = true;

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
		if(value < -1.0D) {
			value = -1.0D;
		}

		if(value > 1.0D) {
			value = 1.0D;
		}

		short intValue = (short)((int)(value * 32767.0D));
		swap16(intValue, buffer);
		return (byte)2;
	}

	private static byte packFXP8_8(byte[] buffer, double value) {
		short intValue = (short)((int)(value * 256.0D));
		swap16(intValue, buffer);
		return (byte)2;
	}

	private static byte packFXP16_16(byte[] buffer, double value) {
		int intValue = (int)(value * 65536.0D);
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
		return (double)unpack16(buffer, 0) / 32767.0D;
	}

	private static double unpackFXP8_8(byte[] buffer) {
		return (double)unpack16(buffer, 0) / 256.0D;
	}

	private static double unpackFXP16_16(byte[] buffer) {
		return (double)unpack32(buffer, 0) / 65536.0D;
	}

	private static short unpackINT16(byte[] buffer) {
		return unpack16(buffer, 0);
	}

	private static int unpackINT32(byte[] buffer) {
		return unpack32(buffer, 0);
	}

	@SuppressWarnings("WeakerAccess")
	public boolean FXP8_EQ(double a, double b) {
		return (int)(a * 256.0D) == (int)(b * 256.0D);
	}

	@SuppressWarnings("WeakerAccess")
	public boolean FXP16_EQ(double a, double b) {
		return (int)(a * 65536.0D) == (int)(b * 65536.0D);
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
		return "GenericCANMotorControllerPart1 ID " + this.m_deviceNumber;
	}

	public int getDeviceID() {
		return this.m_deviceNumber;
	}

	/** @deprecated */
	@Deprecated
	public void stopMotor() {
		this.disableControl();
	}

	public void initTable(ITable subtable) {
		this.m_table = subtable;
		this.updateTable();
	}

	public ITable getTable() {
		return this.m_table;
	}

	public void startLiveWindowMode() {
		this.set(0);
		this.m_table_listener = this.createTableListener();
		this.m_table.addTableListener(this.m_table_listener, true);
	}

	public void stopLiveWindowMode() {
		this.set(0);
		this.m_table.removeTableListener(this.m_table_listener);
	}

	private enum LimitMode {
		SwitchInputsOnly((byte)0),
		SoftPositionLimits((byte)1);

		public final byte value;

		public static LimitMode valueOf(byte value) {
			LimitMode[] var1 = values();

			for (LimitMode mode : var1) {
				if (mode.value == value) {
					return mode;
				}
			}

			return null;
		}

		LimitMode(byte value) {
			this.value = value;
		}
	}

	private enum NeutralMode {
		Jumper((byte)0),
		Brake((byte)1),
		Coast((byte)2);

		public final byte value;

		public static NeutralMode valueOf(byte value) {
			NeutralMode[] var1 = values();

			for (NeutralMode mode : var1) {
				if (mode.value == value) {
					return mode;
				}
			}

			return null;
		}

		NeutralMode(byte value) {
			this.value = value;
		}
	}

	private enum JaguarControlMode implements ControlMode {
		PercentVbus,
		Current,
		Speed,
		Position,
		Voltage;

		public boolean isPID() {
			return this == Current || this == Speed || this == Position;
		}

		public int getValue() {
			return this.ordinal();
		}
	}

	private static class PotentiometerTag {
		private PotentiometerTag() {
		}
	}

	private static class QuadEncoderTag {
		private QuadEncoderTag() {
		}
	}

	private static class EncoderTag {
		private EncoderTag() {
		}
	}
}
