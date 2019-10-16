package frc.robot;

public class TalonPort 
{
	static int intakeTop = 10;			// talon #8, invert = false; breaker #2
	static int intakeBottom = 11;		// talon #9, invert = default; breaker #4
	static int wristMotor = 17;			// spark #17, motor 7; breaker #14
	static int topClimber = 15;			// talon #15; breaker #0
	static int bottomClimber = 16;		// talon #16; breaker #1
	static int frontRightMotor = 8;
	static int centerRightMotor = 9;	// must be inversed
	static int rearRightMotor = 2;		// talon #3, invert = false; tal2; breaker #13
	static int frontLeftMotor = 5;
	static int centerLeftMotor = 4;		// must be inversed
	static int rearLeftMotor = 6;		// talon #2, invert = default; tal4; breaker #3
	static int leftEncoder = 5;
	static int rightEncoder = 8;
}