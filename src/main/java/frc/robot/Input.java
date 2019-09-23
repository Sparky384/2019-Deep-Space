package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Input {
    private static Joystick stick0 = new Joystick(0);
    private static Joystick stick1 = new Joystick(1);
    
	public static boolean getButtonPressed(int stick, int b)	{
		boolean result = false;
		if (stick == 0) {
			result = stick0.getRawButtonPressed(b);
		}
		else if (stick == 1) {
			result = stick1.getRawButton(b);
		}
		return result;
	}
	
	public static boolean getButtonHeld(int stick, int b)	{
		boolean result = false;
		
		if (stick == 0) {
			result = stick0.getRawButton(b);
		}
		
		if (stick == 1) {
			result = stick1.getRawButton(b);
		}
		return result;
	}
	
	public static boolean getButtonReleased(int stick, int b) {
		boolean result = false;
		if (stick == 0) {
			result = stick0.getRawButtonReleased(b);
		}
		else if (stick == 1) {
			result = stick1.getRawButtonReleased(b);
		}
		return result;
	}
	
	public static double getAxis(int stick, int axis) {
		double axisval = 0.0;

		if (stick == 0) {
			if (axis == 0) {
				axisval = stick0.getX();
			} else if (axis == 1) {
				axisval = stick0.getY();
			} else if (axis == 2)
				axisval = stick0.getZ();
		} else if (stick == 1) {
			if (axis == 0) {
				axisval = stick1.getX();
			} else if (axis == 1) {
				axisval = stick1.getY();
			} else if (axis == 2)
				axisval = stick1.getZ();
		} else {	// an illegal value was entered
			axisval = 0.0;
		}
		return axisval;
	}
}