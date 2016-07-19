package org.firstinspires.frc.framework.granulation;

import edu.wpi.first.wpilibj.SafePWM;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.hal.DIOJNI;
import org.firstinspires.frc.framework.abstraction.RioHWPort;
import org.firstinspires.frc.framework.abstraction.RioHWPort.*;
import org.firstinspires.frc.framework.hardware.MotorController;
import org.firstinspires.frc.framework.hardware.MotorControllerType;

/**
 * Clutter-free replacement for the classes Jaguar, SD540, Spark, Talon, TalonSRX, Victor and VictorSP from WPILibJ v2016.0.0.
 * Creating an object to manipulate motor controllers? You probably want MotorController.
 * Jaguar.java, SD540.java, Spark.java, Talon.java, TalonSRX.java, Victor.java and VictorSP.java (all decompiled) total to < TODO lines. This file is < 150 lines.
 * @author FRC 4739 Thunderbolts Robotics
 * @see MotorController
 * @version 2016-07-10/00
 */
public class GenericPWMMotorController extends SafePWM implements SpeedController {
	private enum InitPresets {
		Jaguar(new double[]{2.31, 1.55, 1.507, 1.454, 0.697}, false),
		SD540(new double[]{2.05, 1.55, 1.5, 1.44, 0.94}, false),
		Spark(new double[]{2.003, 1.55, 1.5, 1.46, 0.999}, false),
		Talon(new double[]{2.037, 1.539, 1.513, 1.487, 0.989}, false),
		TalonSRX(new double[]{2.004, 1.52, 1.5, 1.48, 0.997}, false),
		Victor(new double[]{2.027, 1.525, 1.507, 1.49, 1.026}, true),
		VictorSP(new double[]{2.004, 1.52, 1.5, 1.48, 0.997}, false);

		public static final double[] DEFAULT_ARGS = new double[]{2, 1.5, 1.5, 1.5, 1};
		private final double[] args;
		private final PeriodMultiplier multiplier;
		InitPresets(double[] d, boolean is2X) {
			args = d;
			multiplier = is2X ? PeriodMultiplier.k2X : PeriodMultiplier.k1X;
		}
		public double[] getArgs() {
			return args;
		}
		public PeriodMultiplier getMultiplier() {
			return multiplier;
		}
	}

	private boolean isInverted = false;

	private static final int PWM_CENTER_VALUE = 999;
	private final int PWMMaxPositiveValue;
	private final int PWMMaxNegativeValue;

	private GenericPWMMotorController(int i, MotorControllerType m) {
		super(i);
		double[] args;
		switch (m) {
			case Jaguar:
				args = InitPresets.Jaguar.getArgs();
				setPeriodMultiplier(InitPresets.Jaguar.getMultiplier());
				break;
			case SD540:
				args = InitPresets.SD540.getArgs();
				setPeriodMultiplier(InitPresets.SD540.getMultiplier());
				break;
			case Spark:
				args = InitPresets.Spark.getArgs();
				setPeriodMultiplier(InitPresets.Spark.getMultiplier());
				break;
			case Talon:
			case TalonSR:
				args = InitPresets.Talon.getArgs();
				setPeriodMultiplier(InitPresets.Talon.getMultiplier());
				break;
			case TalonSRX:
				args = InitPresets.TalonSRX.getArgs();
				setPeriodMultiplier(InitPresets.TalonSRX.getMultiplier());
				break;
			case Victor:
				args = InitPresets.Victor.getArgs();
				setPeriodMultiplier(InitPresets.Victor.getMultiplier());
				break;
			case VictorSP:
				args = InitPresets.VictorSP.getArgs();
				setPeriodMultiplier(InitPresets.VictorSP.getMultiplier());
				break;
			default:
				args = InitPresets.DEFAULT_ARGS;
				setPeriodMultiplier(PeriodMultiplier.k1X);
		}
		setBounds(args[0], args[1], args[2], args[3], args[4]);
		double loopTime = DIOJNI.getLoopTiming() / 40000;
		PWMMaxPositiveValue = (int) ((args[0] - 1.5) / loopTime + 999);
		PWMMaxNegativeValue = (int) ((args[4] - 1.5) / loopTime + 999);
		setRaw(PWM_CENTER_VALUE);
		setZeroLatch();

		System.out.println("loopTime: " + loopTime);
	}

	public static GenericPWMMotorController builder(RioHWPort p, MotorControllerType m) {
		if (p.getType() == PortType.PWM) {
			return new GenericPWMMotorController(p.getIndex(), m);
		} else {
			throw new MismatchedRioPortException(PortType.PWM, p.getType());
		}
	}

	public void set(double d) {
		d = Math.min(Math.max(isInverted ? -d : d, -1), 1);
		if (d == 0) {
			this.setRaw(PWM_CENTER_VALUE);
		} else if (d > 0) {
			int minPosPWM = PWM_CENTER_VALUE + 1;
			//TODO what is this calculation
			this.setRaw((int) (d * (PWMMaxPositiveValue - minPosPWM) + minPosPWM + 0.5));
		} else {
			int maxNegPWM = PWM_CENTER_VALUE - 1;
			//TODO what is this calculation
			this.setRaw((int) (d * (maxNegPWM - PWMMaxNegativeValue) + maxNegPWM + 0.5));
		}
		this.Feed();
	}

	public void set(double d, byte b) {
		set(d);
	}
	public void setInverted(boolean b) {
		isInverted = b;
	}
	public boolean getInverted() {
		return isInverted;
	}
	public double get() {
		return this.getSpeed();
	}
	public void pidWrite(double output) {
		this.set(output);
	}
}
