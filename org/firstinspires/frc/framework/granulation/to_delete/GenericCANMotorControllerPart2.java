package org.firstinspires.frc.framework.granulation.to_delete;

import edu.wpi.first.wpilibj.CANSpeedController;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDeviceStatus;
import edu.wpi.first.wpilibj.CANTalon.SetValueMotionProfile;
import edu.wpi.first.wpilibj.CANTalon.StatusFrameRate;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.hal.CanTalonJNI;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;
import org.firstinspires.frc.framework.hardware.MotorController_Individual;

public class GenericCANMotorControllerPart2 implements MotorSafety, PIDOutput, CANSpeedController {
	private final MotorSafetyHelper m_safetyHelper;
	private boolean isInverted = false;
	private PIDSourceType m_pidSource;
	private long m_handle;
	private TalonControlMode m_controlMode;
	private static final double kDelayForSolicitedSignals = 0.004D;
	private final int m_deviceNumber;
	private boolean m_controlEnabled;
	private int m_profile;
	private double m_setPoint;
	private int m_codesPerRev;
	private int m_numPotTurns;
	private FeedbackDevice m_feedbackDevice;
	private ITable m_table;
	private ITableListener m_table_listener;

	public GenericCANMotorControllerPart2(int deviceNumber) {
		this.m_pidSource = PIDSourceType.kDisplacement;
		this.m_table = null;
		this.m_table_listener = null;
		this.m_deviceNumber = deviceNumber;
		this.m_handle = CanTalonJNI.new_CanTalonSRX(deviceNumber);
		this.m_safetyHelper = new MotorSafetyHelper(this);
		this.m_controlEnabled = true;
		this.m_profile = 0;
		this.m_setPoint = 0;
		this.m_codesPerRev = 0;
		this.m_numPotTurns = 0;
		this.m_feedbackDevice = FeedbackDevice.QuadEncoder;
		this.setProfile(this.m_profile);
		this.applyControlMode(TalonControlMode.PercentVbus);
		LiveWindow.addActuator("CANTalon", this.m_deviceNumber, this);
	}

	public GenericCANMotorControllerPart2(int deviceNumber, int controlPeriodMs) {
		this.m_pidSource = PIDSourceType.kDisplacement;
		this.m_table = null;
		this.m_table_listener = null;
		this.m_deviceNumber = deviceNumber;
		this.m_handle = CanTalonJNI.new_CanTalonSRX(deviceNumber, controlPeriodMs);
		this.m_safetyHelper = new MotorSafetyHelper(this);
		this.m_controlEnabled = true;
		this.m_profile = 0;
		this.m_setPoint = 0;
		this.m_codesPerRev = 0;
		this.m_numPotTurns = 0;
		this.m_feedbackDevice = FeedbackDevice.QuadEncoder;
		this.setProfile(this.m_profile);
		this.applyControlMode(TalonControlMode.PercentVbus);
		LiveWindow.addActuator("CANTalon", this.m_deviceNumber, this);
	}

	public GenericCANMotorControllerPart2(int deviceNumber, int controlPeriodMs, int enablePeriodMs) {
		this.m_pidSource = PIDSourceType.kDisplacement;
		this.m_table = null;
		this.m_table_listener = null;
		this.m_deviceNumber = deviceNumber;
		this.m_handle = CanTalonJNI.new_CanTalonSRX(deviceNumber, controlPeriodMs, enablePeriodMs);
		this.m_safetyHelper = new MotorSafetyHelper(this);
		this.m_controlEnabled = true;
		this.m_profile = 0;
		this.m_setPoint = 0;
		this.m_codesPerRev = 0;
		this.m_numPotTurns = 0;
		this.m_feedbackDevice = FeedbackDevice.QuadEncoder;
		this.setProfile(this.m_profile);
		this.applyControlMode(TalonControlMode.PercentVbus);
		LiveWindow.addActuator("CANTalon", this.m_deviceNumber, this);
	}

	public void pidWrite(double output) {
		if(this.getControlMode() == TalonControlMode.PercentVbus) {
			this.set(output);
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
		return this.getPosition();
	}

	public void delete() {
		this.disable();
		if(this.m_handle != 0L) {
			CanTalonJNI.delete_CanTalonSRX(this.m_handle);
			this.m_handle = 0L;
		}

	}

	public void set(double outputValue) {
		this.m_safetyHelper.feed();
		if(this.m_controlEnabled) {
			this.m_setPoint = outputValue;
			switch(this.m_controlMode.ordinal()) {
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

			CanTalonJNI.SetModeSelect(this.m_handle, this.m_controlMode.value);
		}

	}

	public void setInverted(boolean isInverted) {
		this.isInverted = isInverted;
	}

	public boolean getInverted() {
		return this.isInverted;
	}

	public void set(double outputValue, byte thisValueDoesNotDoAnything) {
		this.set(outputValue);
	}

	public void reset() {
		this.disable();
		this.clearIAccum();
	}

	public boolean isEnabled() {
		return this.isControlEnabled();
	}

	public double getError() {
		return (double)this.getClosedLoopError();
	}

	public void setSetpoint(double setpoint) {
		this.set(setpoint);
	}

	public void reverseSensor(boolean flip) {
		CanTalonJNI.SetRevFeedbackSensor(this.m_handle, flip?1:0);
	}

	public void reverseOutput(boolean flip) {
		CanTalonJNI.SetRevMotDuringCloseLoopEn(this.m_handle, flip?1:0);
	}

	public double get() {
		switch(this.m_controlMode.ordinal()) {
			case 1:
			case 2:
			default:
				return (double)CanTalonJNI.GetAppliedThrottle(this.m_handle) / 1023;
			case 3:
				return this.getOutputVoltage();
			case 4:
				return this.ScaleNativeUnitsToRpm(this.m_feedbackDevice, (long)CanTalonJNI.GetSensorVelocity(this.m_handle));
			case 5:
				return this.ScaleNativeUnitsToRotations(this.m_feedbackDevice, CanTalonJNI.GetSensorPosition(this.m_handle));
			case 6:
				return this.getOutputCurrent();
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
		FeedbackDeviceStatus retval = FeedbackDeviceStatus.FeedbackStatusUnknown;
		switch(feedbackDevice.ordinal()) {
			case 6:
			case 7:
			case 8:
				if(CanTalonJNI.IsPulseWidthSensorPresent(this.m_handle) == 0) {
					retval = FeedbackDeviceStatus.FeedbackStatusNotPresent;
				} else {
					retval = FeedbackDeviceStatus.FeedbackStatusPresent;
				}
			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
			default:
				return retval;
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
		if(this.m_profile == 0) {
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

	public void configEncoderCodesPerRev(int codesPerRev) {
		this.m_codesPerRev = codesPerRev;
		this.setParameter(CanTalonJNI.param_t.eNumberEncoderCPR, (double)this.m_codesPerRev);
	}

	public void configPotentiometerTurns(int turns) {
		this.m_numPotTurns = turns;
		this.setParameter(CanTalonJNI.param_t.eNumberPotTurns, (double)this.m_numPotTurns);
	}

	public double getTemperature() {
		return CanTalonJNI.GetTemp(this.m_handle);
	}

	public double getOutputCurrent() {
		return CanTalonJNI.GetCurrent(this.m_handle);
	}

	public double getOutputVoltage() {
		return this.getBusVoltage() * (double)CanTalonJNI.GetAppliedThrottle(this.m_handle) / 1023;
	}

	public double getBusVoltage() {
		return CanTalonJNI.GetBatteryV(this.m_handle);
	}

	public double getPosition() {
		return this.ScaleNativeUnitsToRotations(this.m_feedbackDevice, CanTalonJNI.GetSensorPosition(this.m_handle));
	}

	public void setPosition(double pos) {
		int nativePos = this.ScaleRotationsToNativeUnits(this.m_feedbackDevice, pos);
		CanTalonJNI.SetSensorPosition(this.m_handle, nativePos);
	}

	public double getSpeed() {
		return this.ScaleNativeUnitsToRpm(this.m_feedbackDevice, (long)CanTalonJNI.GetSensorVelocity(this.m_handle));
	}

	public TalonControlMode getControlMode() {
		return this.m_controlMode;
	}

	public void setControlMode(int mode) {
		this.changeControlMode(TalonControlMode.valueOf(mode));
	}

	private void applyControlMode(TalonControlMode controlMode) {
		this.m_controlMode = controlMode;
		if(controlMode == TalonControlMode.Disabled) {
			this.m_controlEnabled = false;
		}

		CanTalonJNI.SetModeSelect(this.m_handle, TalonControlMode.Disabled.value);
		UsageReporting.report(52, this.m_deviceNumber + 1, controlMode.value);
	}

	public void changeControlMode(TalonControlMode controlMode) {
		if(this.m_controlMode != controlMode) {
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

	public void enableControl() {
		this.changeControlMode(this.m_controlMode);
		this.m_controlEnabled = true;
	}

	public void enable() {
		this.enableControl();
	}

	public void disableControl() {
		CanTalonJNI.SetModeSelect(this.m_handle, TalonControlMode.Disabled.value);
		this.m_controlEnabled = false;
	}

	public boolean isControlEnabled() {
		return this.m_controlEnabled;
	}

	public double getP() {
		if(this.m_profile == 0) {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot0_P.value);
		} else {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot1_P.value);
		}

		Timer.delay(kDelayForSolicitedSignals);
		return CanTalonJNI.GetPgain(this.m_handle, this.m_profile);
	}

	public double getI() {
		if(this.m_profile == 0) {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot0_I.value);
		} else {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot1_I.value);
		}

		Timer.delay(kDelayForSolicitedSignals);
		return CanTalonJNI.GetIgain(this.m_handle, this.m_profile);
	}

	public double getD() {
		if(this.m_profile == 0) {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot0_D.value);
		} else {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot1_D.value);
		}

		Timer.delay(kDelayForSolicitedSignals);
		return CanTalonJNI.GetDgain(this.m_handle, this.m_profile);
	}

	public double getF() {
		if(this.m_profile == 0) {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot0_F.value);
		} else {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot1_F.value);
		}

		Timer.delay(kDelayForSolicitedSignals);
		return CanTalonJNI.GetFgain(this.m_handle, this.m_profile);
	}

	public double getIZone() {
		if(this.m_profile == 0) {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot0_IZone.value);
		} else {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot1_IZone.value);
		}

		Timer.delay(kDelayForSolicitedSignals);
		return (double)CanTalonJNI.GetIzone(this.m_handle, this.m_profile);
	}

	public double getCloseLoopRampRate() {
		if(this.m_profile == 0) {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot0_CloseLoopRampRate.value);
		} else {
			CanTalonJNI.RequestParam(this.m_handle, CanTalonJNI.param_t.eProfileParamSlot1_CloseLoopRampRate.value);
		}

		Timer.delay(kDelayForSolicitedSignals);
		double throttlePerMs = (double)CanTalonJNI.GetCloseLoopRampRate(this.m_handle, this.m_profile);
		return throttlePerMs / 1023 * 12 * 1000;
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

	public void setP(double p) {
		CanTalonJNI.SetPgain(this.m_handle, this.m_profile, p);
	}

	public void setI(double i) {
		CanTalonJNI.SetIgain(this.m_handle, this.m_profile, i);
	}

	public void setD(double d) {
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

	public void setVoltageRampRate(double rampRate) {
		int rate = (int)(rampRate * 1023 / 12 / 100);
		CanTalonJNI.SetRampThrottle(this.m_handle, rate);
	}

	public void setVoltageCompensationRampRate(double rampRate) {
		CanTalonJNI.SetVoltageCompensationRate(this.m_handle, rampRate / 1000);
	}

	public void ClearIaccum() {
		CanTalonJNI.SetParam(this.m_handle, CanTalonJNI.param_t.ePidIaccum.value, 0);
	}

	@SuppressWarnings("SameParameterValue")
	public void setPID(double p, double i, double d, double f, int izone, double closeLoopRampRate, int profile) {
		if(profile != 0 && profile != 1) {
			throw new IllegalArgumentException("Talon PID profile must be 0 or 1.");
		} else {
			this.m_profile = profile;
			this.setProfile(profile);
			this.setP(p);
			this.setI(i);
			this.setD(d);
			this.setF(f);
			this.setIZone(izone);
			this.setCloseLoopRampRate(closeLoopRampRate);
		}
	}

	public void setPID(double p, double i, double d) {
		this.setPID(p, i, d, 0, 0, 0, this.m_profile);
	}

	public double getSetpoint() {
		return this.m_setPoint;
	}

	public void setProfile(int profile) {
		if(profile != 0 && profile != 1) {
			throw new IllegalArgumentException("Talon PID profile must be 0 or 1.");
		} else {
			this.m_profile = profile;
			CanTalonJNI.SetProfileSlotSelect(this.m_handle, this.m_profile);
		}
	}

	/** @deprecated */
	@Deprecated
	public void stopMotor() {
		this.disableControl();
	}

	public void disable() {
		this.disableControl();
	}

	public int getDeviceID() {
		return this.m_deviceNumber;
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

	public void configMaxOutputVoltage(double voltage) {
		this.configPeakOutputVoltage(voltage, -voltage);
	}

	public void configPeakOutputVoltage(double forwardVoltage, double reverseVoltage) {
		if(forwardVoltage > 12) {
			forwardVoltage = 12;
		} else if(forwardVoltage < 0) {
			forwardVoltage = 0;
		}

		if(reverseVoltage > 0) {
			reverseVoltage = 0;
		} else if(reverseVoltage < -12) {
			reverseVoltage = -12;
		}

		this.setParameter(CanTalonJNI.param_t.ePeakPosOutput, 1023 * forwardVoltage / 12);
		this.setParameter(CanTalonJNI.param_t.ePeakNegOutput, 1023 * reverseVoltage / 12);
	}

	public void configNominalOutputVoltage(double forwardVoltage, double reverseVoltage) {
		if(forwardVoltage > 12) {
			forwardVoltage = 12;
		} else if(forwardVoltage < 0) {
			forwardVoltage = 0;
		}

		if(reverseVoltage > 0) {
			reverseVoltage = 0;
		} else if(reverseVoltage < -12) {
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
		double retval = 0;
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
						retval = 4096;
						scalingAvail = true;
				}

				if(!scalingAvail && 0 != this.m_codesPerRev) {
					retval = (double)(4 * this.m_codesPerRev);
					scalingAvail = true;
				}
				break;
			case 2:
			case 3:
				if(0 != this.m_numPotTurns) {
					retval = 1024 / (double)this.m_numPotTurns;
					scalingAvail = true;
				}
				break;
			case 4:
			case 5:
				if(0 != this.m_codesPerRev) {
					retval = (double)(this.m_codesPerRev);
					scalingAvail = true;
				}
				break;
			case 6:
			case 7:
			case 8:
				retval = 4096;
				scalingAvail = true;
		}

		return !scalingAvail?0:retval;
	}

	private int ScaleRotationsToNativeUnits(FeedbackDevice devToLookup, double fullRotations) {
		int retval = (int)fullRotations;
		double scalar = this.GetNativeUnitsPerRotationScalar(devToLookup);
		if(scalar > 0) {
			retval = (int)(fullRotations * scalar);
		}

		return retval;
	}

	private int ScaleVelocityToNativeUnits(FeedbackDevice devToLookup, double rpm) {
		int retval = (int)rpm;
		double scalar = this.GetNativeUnitsPerRotationScalar(devToLookup);
		if(scalar > 0) {
			retval = (int)(rpm * 0.0016666666666666668D * scalar);
		}

		return retval;
	}

	private double ScaleNativeUnitsToRotations(FeedbackDevice devToLookup, int nativePos) {
		double retval = (double)nativePos;
		double scalar = this.GetNativeUnitsPerRotationScalar(devToLookup);
		if(scalar > 0) {
			retval = (double)nativePos / scalar;
		}

		return retval;
	}

	private double ScaleNativeUnitsToRpm(FeedbackDevice devToLookup, long nativeVel) {
		double retval = (double)nativeVel;
		double scalar = this.GetNativeUnitsPerRotationScalar(devToLookup);
		if(scalar > 0) {
			retval = (double)nativeVel / (scalar * 0.0016666666666666668D);
		}

		return retval;
	}

	public void enableZeroSensorPositionOnIndex(boolean enable, boolean risingEdge) {
		if(enable) {
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
		if(this.isMotionProfileTopLevelBufferFull()) {
			return false;
		} else {
			int targPos = this.ScaleRotationsToNativeUnits(this.m_feedbackDevice, trajPt.position);
			int targVel = this.ScaleVelocityToNativeUnits(this.m_feedbackDevice, trajPt.velocity);
			int profileSlotSelect = trajPt.profileSlotSelect > 0?1:0;
			int timeDurMs = trajPt.timeDurMs;
			if(timeDurMs > 255) {
				timeDurMs = 255;
			}

			if(timeDurMs < 0) {
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

	public void getMotionProfileStatus(MotionProfileStatus motionProfileStatus) {
		CanTalonJNI.GetMotionProfileStatus(this.m_handle, this, motionProfileStatus);
	}

	protected void setMotionProfileStatusFromJNI(MotionProfileStatus motionProfileStatus, int flags, int profileSlotSelect, int targPos, int targVel, int topBufferRem, int topBufferCnt, int btmBufferCnt, int outputEnable) {
		motionProfileStatus.topBufferRem = topBufferRem;
		motionProfileStatus.topBufferCnt = topBufferCnt;
		motionProfileStatus.btmBufferCnt = btmBufferCnt;
		motionProfileStatus.hasUnderrun = (flags & 2) > 0;
		motionProfileStatus.isUnderrun = (flags & 4) > 0;
		motionProfileStatus.activePointValid = (flags & 1) > 0;
		motionProfileStatus.activePoint.isLastPoint = (flags & 8) > 0;
		motionProfileStatus.activePoint.velocityOnly = (flags & 16) > 0;
		motionProfileStatus.activePoint.position = this.ScaleNativeUnitsToRotations(this.m_feedbackDevice, targPos);
		motionProfileStatus.activePoint.velocity = this.ScaleNativeUnitsToRpm(this.m_feedbackDevice, (long)targVel);
		motionProfileStatus.activePoint.profileSlotSelect = profileSlotSelect;
		motionProfileStatus.outputEnable = SetValueMotionProfile.valueOf(outputEnable);
		motionProfileStatus.activePoint.zeroPos = false;
		motionProfileStatus.activePoint.timeDurMs = 0;
	}

	public void clearMotionProfileHasUnderrun() {
		this.setParameter(CanTalonJNI.param_t.eMotionProfileHasUnderrunErr, 0);
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
		return "CANTalon ID " + this.m_deviceNumber;
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

	private static class MotionProfileStatus {
		int topBufferRem;
		int topBufferCnt;
		int btmBufferCnt;
		boolean hasUnderrun;
		boolean isUnderrun;
		boolean activePointValid;
		final TrajectoryPoint activePoint = new TrajectoryPoint();
		SetValueMotionProfile outputEnable;
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
