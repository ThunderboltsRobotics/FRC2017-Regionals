package org.firstinspires.frc.framework.abstraction;

/**
 * Sometimes when programming for FRC, the IDE just doesn't understand that System.exit() means 'stop running'. For example, IDEA seems to think that when you're switching on an enum (stored in private final EnumName) that you've filtered in the constructor, you need a default case with a return value. That's what this is for. Just write:
 * default:
 *     return RobotKiller.{return type}Kill()
 * ...after the last case, and all of your 'problems' will vanish.
 * Alternatively, confuse the other members of your software team by omitting the break before this default case, crashing the robot.
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-13/01
 */
public class RobotKiller {
	private static void voidKill() {
		System.exit(0);
	}
	public static boolean booleanKill() {
		voidKill();
		return false;
	}
	public static byte byteKill() {
		voidKill();
		return 0;
	}
	public static double doubleKill() {
		return byteKill();
	}
	public static float floatKill() {
		return byteKill();
	}
	public static int intKill() {
		return byteKill();
	}
	public static long longKill() {
		return byteKill();
	}
	public static String stringKill() {
		voidKill();
		return "";
	}
}
