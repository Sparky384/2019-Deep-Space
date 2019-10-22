/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class Constants{
	public static final double ENCODER_COUNTS_PER_ANGLE = 10.53; // 5.73
	public static final double CLIMBER_RAMPUP = 1;
	public static final double kSensorUnitsPerRotation = 512;
	public static final int kBaseTrajPeriodMs = 0;
	public static final double kNeutralDeadband = 0.01;
	
	public static final double ERROR_VALUE = 1000000.000000;
	public static final int MAX_SIZE = 5;
	public static final int MAX_MISSED_DATA = 25;
	public static final double ANGLE_THRESHOLD = 3.0; 
	public static final double DIST_THRESHOLD = 7.0;

	// values for prototype: 55, 105, 215, 240, 0, 250, 345
	public static final int WRIST_HI = 55;
	public static final int WRIST_LOW = 105;
	public static final int WRIST_HATCH = 215;
	public static final int WRIST_BOTTOM = 240;
	public static final int WRIST_TOP = 0;
	public static final int WRIST_FLOOR = 245;
	public static final int WRIST_LVL2 = 345;

	// New positions for district champs
	public static final int BALL_INTAKE = 269;
    public static final int HATCH_INTAKE = 255;
    public static final int ROCKET_CARGO = 105;
    public static final int HATCH_SCORE = 215;

	public static final int XAXIS = 0;
	public static final int YAXIS = 1;
	public static final int PILOT = 0;
	public static final int COPILOT = 1;
	
	public static final double kDrive_P = 0.033;		// far away P gain - 0.060   From: 0.009, COMPETITION: 0.033
 	public static final double kDrive_Pa = 0.002;		// approach P gain - 0.08, COMPETITION: 0.002
 	public static final double kDrive_I = 0.0015;		// far away I gain, COMPETITION: 0.0015
 	public static final double kDrive_Ia = 0.00015;		// approach I gain - 0.0005, COMPETITION: 0.00015
	 public static final double kDrive_D = 0.0;			// COMPETITION: 0.0
	 
	public static final double kDriveTurn_P = 0.045;		// far away P gain - 0.060   From: 0.009, COMPETITION: 0.033
 	public static final double kDriveTurn_Pa = 0.04;		// approach P gain - 0.08, COMPETITION: 0.002
 	public static final double kDriveTurn_I = 0.0015;		// far away I gain, COMPETITION: 0.0015
 	public static final double kDriveTurn_Ia = 0.00015;		// approach I gain - 0.0005, COMPETITION: 0.00015
 	public static final double kDriveTurn_D = 0.01;			// COMPETITION: 0.0
 	
	public static final double kTurn_P = 0.04;			//
	public static final double kTurn_Pa = 0.10;			//
	public static final double kTurn_I = 0.001;			//
	public static final double kTurn_Ia = 0.100;		//
	public static final double kTurn_D = 0.01; 			//
	
	public static final double kWrist_Pu = 0.025; 		// Proportional Gain, rotating upwards [0.005] Was: 0.002
	public static final double kWrist_Iu = 0.0001;			// Integral Gain, rotating upwards [0.0005] Was: 0.1
	public static final double kWrist_Du = 0.0; 		// Derivative Gain, rotating upwards Was: 0
	public static final double kWrist_FFu = 0.00042;		// Feed Forward Gain, rotating upwards [0.35] Was: 0
	
	public static final double kWrist_Pd = 0.01; 		// Proportional Gain, rotating downwards [0.005]
	public static final double kWrist_Id = 0.0;			// Integral Gain, rotating downwards
	public static final double kWrist_Dd = 0.0; 		// Derivative Gain, rotating downwards
	public static final double kWrist_FFd = 0.0; 		// Feed Forward Gain, rotating downwards
	
	public static final double kWrist_Pua = 0.1; 		// Proportional Gain approaching, rotating upwards
	public static final double kWrist_Iua = 0.0; 		// Integral Gain approaching, rotating upwards 
	public static final double kWrist_FFua = 0.0; 		// Derivative Gain approaching, rotating upwards
	
	public static final double kWrist_Pda = 0.1; 		// Proportional Gain approaching, rotating downwards
	public static final double kWrist_Ida = 0.0;		// Integral Gain approaching, rotating downwards
	public static final double kWrist_FFda = 0.0; 		// Derivative Gain approaching, rotating downwards
	
	public static final int kWristToleranceDistance = 2; // Wrist Tolerance in encoder counts [15]
		
	public static final double INTAKE_P = 0.0;
	public static final double INTAKE_I = 0.0;
	public static final double INTAKE_D = 0.0;
	
    static final double kToleranceDegrees = 2.0f;	
 	
 	public static final double kToleranceDistance = 0.5;
 	
 	public static final boolean kLeftSensorPhase = false;
	public static final boolean kRightSensorPhase = false;

	/* choose based on what direction you want to be positive,
		this does not affect motor invert. */
	public static final boolean kLeftMotorInvert = true;
	public static final boolean kRightMotorInvert = true;
	
	public static final double kDriveYaw_P = 0.02;
 	public static final double kDriveYaw_I = 0.0001;
 	public static final double kDriveYaw_D = 0.0;
	
	public static final int kPIDLoopIdx = 0;

    // Current limiting
    // Spark max
    public static final int kSparkCurrentLimit = 60;                // steady state current limit [60]
    public static final int kSparkPeakCurrentLimit = 100;           // peak short duration current limit [100]
    public static final int kSparkPeakCurrentDuration = 200;        // peak short duration current time, milliseconds [200]
	
	// Spark max ramp rate
	public static final double kSparkRampRate = 0.100;

	/*
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 10;
	
	public static final int DRIVE_LEFT = 0;
	public static final int DRIVE_RIGHT = 1;
	public static final int DRIVE_ENC_PPR  = 512;	// Talon counts 4 edges * 128 PPR
	// Reducing wheel diameter make robot go farther for given drive distance
	public static final double DRIVE_DIST_PER_PULSE = 5.875 * Math.PI / DRIVE_ENC_PPR;	// units are in inches
}
