package frc.robot;

public class TalonPort 
{
	static int wristEncoder = 1;		// talon #7, pos = down neg = up; breaker #4
	static int intakeTop = 10;			// talon #8, invert = false; breaker #5
	static int intakeBottom = 11;		// talon #9, invert = default; breaker #6
	static int wristMotor = 17;			// spark #17, motor 7; breaker #7
	static int topClimber = 15;			// talon #15; breaker #0
	static int bottomClimber = 16;		// talon #16; breaker #15
	static int frontRightMotor = 8;		// talon #4, invert = false; tal1; breaker #14
	static int rearRightMotor = 2;		// talon #3, invert = false; tal2; breaker #13
	static int frontLeftMotor = 5;		// talon #1, invert = default; tal3; breaker #2
	static int rearLeftMotor = 6;		// talon #2, invert = default; tal4; breaker #12
	static int rightBoost = 9;			// spark #9; breaker #3
	static int leftBoost = 3;			// spark #3; breaker #1
}