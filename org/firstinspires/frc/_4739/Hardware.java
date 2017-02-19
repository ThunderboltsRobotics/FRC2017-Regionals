package org.firstinspires.frc._4739;

import org.firstinspires.frc.framework.abstraction.DSGamepadIndex;
import org.firstinspires.frc.framework.software.RioCANID;
import org.firstinspires.frc.framework.hardware.RioHWPort;
import org.firstinspires.frc.framework.hardware.MotorController;
import org.firstinspires.frc.framework.hardware.MotorControllerType;
import org.firstinspires.frc.framework.software.hid_controller_layouts.LogitechF310Gamepad;

/**
 * Offseason hardware setup.
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-08-22/00
 */
public class Hardware {
	private static class PWMPorts {
		//TODO switch HW - 0,1 w/ 2,3
		//TODO decide which side of the robot is the front for off-season demonstrations
		// static final RioHWPort driveLeftA = RioHWPort.PWM_0;
		// static final RioHWPort driveLeftB = RioHWPort.PWM_1;
		// static final RioHWPort driveRightA = RioHWPort.PWM_2;
		// static final RioHWPort driveRightB = RioHWPort.PWM_3;
		// static final RioHWPort shooterBack = RioHWPort.PWM_4;
		// static final RioHWPort shooterLeft = RioHWPort.PWM_5;
		// static final RioHWPort shooterRight = RioHWPort.PWM_6;
		//static final RioHWPort LEDRing = RioHWPort.PWM_9;
		
		// 2017 pre-regional
		static final RioHWPort driveLeft = RioHWPort.PWM_0; // TalonSR (x2, splitter)
		static final RioHWPort driveRight = RioHWPort.PWM_1; // TalonSR (x2, splitter)
		static final RioHWPort driveStrafeA = RioHWPort.PWM_2; // Spark
		static final RioHWPort driveStrafeB = RioHWPort.PWM_3; // Spark
	}
	private static class CANIDs {
		// static final RioCANID shooterAim = new RioCANID(0);
	}

	public static class Drive {
		// private static final MotorController leftA = new MotorController(PWMPorts.driveLeftA, MotorControllerType.TalonSR);
		// private static final MotorController leftB = new MotorController(PWMPorts.driveLeftB, MotorControllerType.TalonSR);
		// private static final MotorController rightA = new MotorController(PWMPorts.driveRightA, MotorControllerType.TalonSR);
		// public static void left(double d) {
		// 	leftA.setSpeed(d);
		// 	leftB.setSpeed(d);
		// }
		// public static void right(double d) {
		// 	rightA.setSpeed(d);
		// 	rightB.setSpeed(d);
		// }
		// @SuppressWarnings("SameParameterValue")
		// public static void drive(double l, double r) {
		// 	left(l);
		// 	right(r);
		// }
		// public static void stop() {
		// 	left(0);
		// 	right(0);
		// }
		
		// 2017 pre-regional
		private static final MotorController left = new MotorController(PWMPorts.driveLeft, MotorControllerType.TalonSR);
		private static final MotorController right = new MotorController(PWMPorts.driveRight, MotorControllerType.TalonSR);
		private static final MotorController strafeA = new MotorController(PWMPorts.driveStrafeA, MotorControllerType.Spark);
		private static final MotorController strafeB = new MotorController(PWMPorts.driveStrafeB, MotorControllerType.Spark);
		static {
			right.reverse();
			strafeA.reverse();
		}
		
		public static void stopDrive() {
			forward(0);
		}
		public static void forward(double speed) {
			left.setSpeed(speed);
			right.setSpeed(speed);
		}
		public static void backward(double speed) {
			left.setSpeed(-speed);
			right.setSpeed(-speed);
		}
		public static void turnCCW(double speed) {
			left.setSpeed(-speed);
			right.setSpeed(speed);
		}
		public static void turnCW(double speed) {
			left.setSpeed(speed);
			right.setSpeed(-speed);
		}
		
		public static void stopStrafe() {
			strafeLeft(0);
		}
		public static void strafeLeft(double speed) {
			strafeA.setSpeed(speed);
			strafeB.setSpeed(speed);
		}
		public static void strafeRight(double speed) {
			strafeA.setSpeed(-speed);
			strafeB.setSpeed(-speed);
		}
	}

	public static class Shooter {
		// private static final double FIRE_SPEED = 1;
		// private static final double INTAKE_SPEED = -0.3;
		// private static final double KICKER_SPEED = 1;
		// private static final double AIM_MAX_SPEED = 1;

		// private static final MotorController aim;
		// static {
		// 	MotorController temp;
		// 	try {
		// 		temp = new MotorController(CANIDs.shooterAim, MotorControllerType.TalonSRX);
		// 	} catch (MotorController.CANIsUnsupportedException e) {
		// 		e.printStackTrace();
		// 		temp = null;
		// 		//Graceful shutdown
		// 		System.exit(0);
		// 	}
		// 	aim = temp;
		// }

		// private static final MotorController back = new MotorController(PWMPorts.shooterBack, MotorControllerType.Victor);
		// private static final MotorController left = new MotorController(PWMPorts.shooterLeft, MotorControllerType.Victor);
		// private static final MotorController right = new MotorController(PWMPorts.shooterRight, MotorControllerType.Victor);
		//private static final MotorEncoder aimEncoder = new MotorEncoder(0);

		// public static void aim(double d) {
		// 	relAim(Math.sqrt(aimEncoder.getPos() - d) * AIM_MAX_SPEED);
		// 	relAim(Math.sqrt(-d) * AIM_MAX_SPEED);
		// }
		// public static void relAim(double d) {
		// 	Shooter.aim.setSpeed(d);
		// }
		// private static void run(double d) {
		// 	//TODO IRL check direction
		// 	Shooter.left.setSpeed(d);
		// 	Shooter.right.setSpeed(d);
		// 	Shooter.back.setSpeed(KICKER_SPEED);
		// }
		// public static void fire() {
		// 	run(FIRE_SPEED);
		// }
		// public static void intake() {
		// 	run(INTAKE_SPEED);
		// }
		// public static void stop() {
		// 	run(0);
		// }
	}

	/*
	public static class LEDRing {
		//private static final PWMLEDArray noIdeaWhatThisDoes = new PWMLEDArray(PWMPorts.LEDRing);
		//private static SPI noIdeaWhatThisDoes;
		public static void off() {
			noIdeaWhatThisDoes.set(0);
		}
		public static void on() {
			noIdeaWhatThisDoes.set(1);
		}
		//white - signal
		//red - power (+)
		//black - power (-)
	}
	*/

	static void initialize() {
		// if (!Drive.rightA.isReversed()) {
		// 	Drive.rightA.reverse();
		// }
		// if (!Drive.rightB.isReversed()) {
		// 	Drive.rightB.reverse();
		// }
		// if (!Shooter.right.isReversed()) {
		// 	Shooter.right.reverse();
		// }
	}

	public static final LogitechF310Gamepad gamepad1 = new LogitechF310Gamepad(new DSGamepadIndex(0));
	public static final LogitechF310Gamepad gamepad2 = new LogitechF310Gamepad(new DSGamepadIndex(1));
}
