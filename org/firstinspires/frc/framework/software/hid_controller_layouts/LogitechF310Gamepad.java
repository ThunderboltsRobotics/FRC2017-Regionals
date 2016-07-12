package org.firstinspires.frc.framework.software.hid_controller_layouts;

import org.firstinspires.frc.framework.abstraction.DSGamepadIndex;

/**
 * Logitech's generic gamepad, modelled after the XBOX 360 and PS3 controllers. Commonly used by FRC teams.
 * @author FRC 4739 Thunderbolts Robotics
 * @version 2016-07-10/00
 */
@SuppressWarnings("unused")
public class LogitechF310Gamepad extends XBOX360Controller {
	public LogitechF310Gamepad(DSGamepadIndex i) {
		super(i);
	}
}
