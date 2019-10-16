package frc.robot;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

// Spark Max libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class Drivetrain {

	public WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(TalonPort.rightEncoder);	// needed for encoder
	//public WPI_TalonSRX rearRightMotor = new WPI_TalonSRX(TalonPort.rearRightMotor);
	public WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(TalonPort.leftEncoder);		// -1.0 to run forward, needed for encoder
	//public WPI_TalonSRX rearLeftMotor = new WPI_TalonSRX(TalonPort.rearLeftMotor);		// -1.0 to run forward
	
	private CANSparkMax frontRightSpark = new CANSparkMax(TalonPort.frontRightMotor, MotorType.kBrushless);
	private CANSparkMax centerRightSpark = new CANSparkMax(TalonPort.centerRightMotor, MotorType.kBrushless);
	private CANSparkMax rearRightSpark = new CANSparkMax(TalonPort.rearRightMotor, MotorType.kBrushless);
	private CANSparkMax frontLeftSpark = new CANSparkMax(TalonPort.frontLeftMotor, MotorType.kBrushless);
	private CANSparkMax centerLeftSpark = new CANSparkMax(TalonPort.centerLeftMotor, MotorType.kBrushless);
	private CANSparkMax rearLeftSpark = new CANSparkMax(TalonPort.rearLeftMotor, MotorType.kBrushless);

	// Need to check elapsed time for PID control
	private Timer intervalTimer = new Timer();	// PID interval timer
	private Timer failTimer = new Timer();	// PID fails if value exceeded 
	private Timer turnTimer = new Timer(); //Used in the turnToAngle
	private boolean timing;
	// Define the motors that are slaved as a control group
	//private SpeedControllerGroup leftDrivetrain = new SpeedControllerGroup(frontLeftMotor, rearLeftMotor);
	//private SpeedControllerGroup rightDrivetrain = new SpeedControllerGroup(frontRightMotor, rearRightMotor);
	private SpeedControllerGroup leftDrivetrain = new SpeedControllerGroup(frontLeftSpark, centerLeftSpark, rearLeftSpark);
	private SpeedControllerGroup rightDrivetrain = new SpeedControllerGroup(frontRightSpark, centerRightSpark, rearRightSpark);
	
	// Set up differential drive for teleop
	private DifferentialDrive diffDrive = new DifferentialDrive(leftDrivetrain, rightDrivetrain);
	
	private AHRS imu;						// the NavX board
	private MiniPID turnController;			// drivebase turning pid controller
	private MiniPID driveController;		// drive distance pid controller
	private MiniPID turnYawController;              // provides steering correction for driveTo() method
	private MiniPID driveTurnController;		// drive distance pid controller
	//private MiniPID turnYawController;		// provides steering correction for driveTo() method
	double rotateToAngleRate;				// PID output for turn PID			
	double driveToDistanceRate;
	double leftStickValue;			
	double rightStickValue;
 	int leftDistanceError;					// drive distance error
 	int rightDistanceError;
 	double currentDistanceError;
	
	public Drivetrain()	{
		initializeEncoders();
		currentDistanceError = 0;
		rightDistanceError = 0;
		leftDistanceError = 0;
		rightStickValue = 0.0;
		leftStickValue = 0.0;
		
		//	Initialize the IMU
		try {
			imu = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
			
		}
		
		frontRightSpark.restoreFactoryDefaults();
		centerRightSpark.restoreFactoryDefaults();
		rearRightSpark.restoreFactoryDefaults();
		frontLeftSpark.restoreFactoryDefaults();
		centerLeftSpark.restoreFactoryDefaults();
		rearLeftSpark.restoreFactoryDefaults();

		// Do we really need this?
		/*frontRightMotor.setSafetyEnabled(false);
		frontLeftMotor.setSafetyEnabled(false);
		rearRightMotor.setSafetyEnabled(false);
		rearLeftMotor.setSafetyEnabled(false);*/
		setBrakeMode(false);
	
		/*
		 * This is the PID controller for the turn
		 */
		turnController = new MiniPID(Constants.kTurn_P, Constants.kTurn_I, Constants.kTurn_D);
		turnController.setOutputLimits(-1.0, 1.0);
		
		/*
		 * This is the PID controller for the drive
		 */
		driveController = new MiniPID(Constants.kDrive_P, Constants.kDrive_I, Constants.kDrive_D);
		driveController.setOutputLimits(-1.0, 1.0);

		driveTurnController = new MiniPID(Constants.kDriveTurn_P, Constants.kDriveTurn_I, Constants.kDriveTurn_D);
		driveTurnController.setOutputLimits(-1.0, 1.0);
		
		turnYawController = new MiniPID (Constants.kDriveYaw_P, Constants.kDriveYaw_I, Constants.kDriveYaw_D);

		/*
		 * This is the PID controller for the Yaw correction to keep the robot driving straight
		 * during the driveTo() method 
		 */
		// = new MiniPID
		//		(Constants.kDriveYaw_P, Constants.kDriveYaw_I, Constants.kDriveYaw_D);
		
		/*
		 * Configure the Talon SRX motor controllers
		 * 
		 */
		frontLeftMotor.configSelectedFeedbackSensor(
				FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		frontRightMotor.configSelectedFeedbackSensor(
				FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		
		/* choose to ensure sensor is positive when output is positive */
		frontLeftMotor.setSensorPhase(Constants.kLeftSensorPhase);
		frontRightMotor.setSensorPhase(Constants.kRightSensorPhase);

		/* choose based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. */ 
	
		//rearRightMotor.setInverted(false);
		//frontRightMotor.setInverted(false);
		rearRightSpark.setInverted(false);
		frontRightMotor.setInverted(false);

		//rearLeftMotor.follow(frontLeftMotor); 	// follow the master talon		
		//rearRightMotor.follow(frontRightMotor);
		centerLeftSpark.follow(frontLeftSpark, true);
		rearLeftSpark.follow(frontLeftSpark);
		centerRightSpark.follow(frontRightSpark, true);
		rearRightSpark.follow(frontRightSpark);
	}
	
	
	/*
	 * Initialize encoders - set to zero
	 */
	public void initializeEncoders()	{
		frontLeftMotor.getSensorCollection().setQuadraturePosition(0, 0);
		frontRightMotor.getSensorCollection().setQuadraturePosition(0, 0);
	}

	/*
	 * Get encoder position
	 * 0 = left side drive rail
	 * 1 = right side drive rail
	 */
	public int getEncoderPosition(int side)	{
		int result;
		
		if(side == 0)	{
			result = frontLeftMotor.getSelectedSensorPosition(0);
		} else if (side == 1)	{
			result = frontRightMotor.getSelectedSensorPosition(0);
		}	else	{	// an illegal value was sent
			result = 0;
		}
		
		return result;
	}

	/*
	 * Get the encoder position as a double, averaging both sides
	 * This returns the distance in inches.
	 * Make sure that both encoder values are of same polarity
	 */
	public double getEncoderDistance()	{
		int leftEncoderDistance;
		int rightEncoderDistance;
		
		leftEncoderDistance = -frontLeftMotor.getSelectedSensorPosition(0);	// negative when forward
		rightEncoderDistance = frontRightMotor.getSelectedSensorPosition(0);
		
		/*
		 * I'm getting half of the pulses that I should on the right encoder, as if 
		 * one of the channels is not working.  I'll use the left only for now
		 * 
		 * Also, left is negative when going forward.
		 */
		
		// Return only the left encoder until the right encoder gets fixed
		return leftEncoderDistance * Constants.DRIVE_DIST_PER_PULSE;
	}

	// for use in turnToEncoders
	public double getEncoderAngle()	{
		int leftEncoderDistance;
		int rightEncoderDistance;
		
		leftEncoderDistance = -frontLeftMotor.getSelectedSensorPosition(0);	// negative when forward
		rightEncoderDistance = frontRightMotor.getSelectedSensorPosition(0);
		
		/*
		 * I'm getting half of the pulses that I should on the right encoder, as if 
		 * one of the channels is not working.  I'll use the left only for now
		 * 
		 * Also, left is negative when going forward.
		 */
		
		// Return only the left encoder until the right encoder gets fixed
		return rightEncoderDistance / Constants.ENCODER_COUNTS_PER_ANGLE;
	}
	
	public double getREncoderDistance() {
		return frontRightMotor.getSelectedSensorPosition(0)*Constants.DRIVE_DIST_PER_PULSE;
	}
	
	public double getLEncoderPosition() {
		return -frontLeftMotor.getSelectedSensorPosition(0);	// negative when forward
	}
	
	/*
	 * Get encoder velocity
	 * 0 = left side drive rail
	 * 1 = right side drive rail
	 * 
	 * Talon reports velocity is sensor units per 100ms. 
	 * Current sensor 128 PPR * 4 = 512 counts per revolution
	 */
	public int getEncoderVelocity(int side)	{
		int result;
		
		if(side == 0)	{
			result = frontLeftMotor.getSelectedSensorVelocity(0);
		} else if (side == 1)	{
			result = frontRightMotor.getSelectedSensorVelocity(0);
		}	else	{	// an illegal value was sent
			result = 0;
		}
		
		return result;
	}

	/*
	 * Return the value of the output to the Talon SRX drive base motor
	 * 0 = front left
	 * 1 = front right
	 */
	public double getDriveMotorOutput(int side)	{
		double result = 0;
		
		if(side == 0)	{
			//result = frontLeftMotor.getMotorOutputPercent();
			result = frontLeftSpark.getAppliedOutput();
		} else if (side == 1)	{
			//result = frontRightMotor.getMotorOutputPercent();
			result = frontRightSpark.getAppliedOutput();
		}
		
		return result;
		
	}
	
	/*
	 * Arcade drive
	 * requires stick number and axis number
	 * 
	 */
	public void arcadeDrive(double speedaxis,  double turnaxis)	{
		diffDrive.arcadeDrive(speedaxis, turnaxis);
	}

	/*
	 * Yaw values are in degrees, clockwise increasing positive
	 * 
	 */
	public float imuGetYaw()	{
		return imu.getYaw();
	}
	

	// FIXME
	public void imuZeroYaw()	{
		imu.zeroYaw();		// this is not working, I get a non-zero value even after calling this method
	}

	/*
	 *  Set the brake mode for the drivetrain
	 *  See  Constants.NEUTRAL_COAST and Constants.NEUTRAL_BRAKE 
	 */
	public void setBrakeMode(boolean brakeon)	{
		if (brakeon)	{
			/*frontRightMotor.setNeutralMode(NeutralMode.Brake);
			frontLeftMotor.setNeutralMode(NeutralMode.Brake); 
			rearRightMotor.setNeutralMode(NeutralMode.Brake);
			frontLeftMotor.setNeutralMode(NeutralMode.Brake);*/
			frontRightSpark.setIdleMode(IdleMode.kBrake);
			frontLeftSpark.setIdleMode(IdleMode.kBrake);
			centerRightSpark.setIdleMode(IdleMode.kBrake);
			centerLeftSpark.setIdleMode(IdleMode.kBrake);
			rearRightSpark.setIdleMode(IdleMode.kBrake);
			rearLeftSpark.setIdleMode(IdleMode.kBrake);
		}	else {
			/*frontRightMotor.setNeutralMode(NeutralMode.Coast);
			frontLeftMotor.setNeutralMode(NeutralMode.Coast);
			rearRightMotor.setNeutralMode(NeutralMode.Coast);
			frontLeftMotor.setNeutralMode(NeutralMode.Coast);*/
			frontRightSpark.setIdleMode(IdleMode.kCoast);
			frontLeftSpark.setIdleMode(IdleMode.kCoast);
			centerRightSpark.setIdleMode(IdleMode.kCoast);
			centerLeftSpark.setIdleMode(IdleMode.kCoast);
			rearRightSpark.setIdleMode(IdleMode.kCoast);
			rearLeftSpark.setIdleMode(IdleMode.kCoast);
		}
	}
	
	/*
	 *  Turns the controller by specified angle.
	 *  Positive angles are clockwise
	 *  Input angle and output ranges are defined in the Drivetrain constructor
	 *  Also requires a timeout value, which is time before we give up PID control and
	 *  move to whatever next step 
	 *  
	 *  Returns:
	 *  0 if not enabled
	 *  1 if enabled and turning
	 *  -1 if error
	 */
	 int turnTo(double angle, double timeout)	{
	 	// This is executed in first call to method
	 	if (!Robot.isTurning) {
			imuZeroYaw();	// I'm gonna zero the angle here
			turnController.setSetpoint(angle);
			rotateToAngleRate = 0; 	// This value will be updated in the pidWrite() method.
			leftStickValue = rightStickValue = 0.0;
			failTimer.start();		// the PID will fail if this timer exceeded
			Robot.isTurning = true;
		}
		
		// This is the final output of the PID
		rotateToAngleRate = turnController.getOutput(imu.getYaw(), angle);
		
		if (angle >= 0.0) {
			leftStickValue = -rotateToAngleRate;
			rightStickValue = rotateToAngleRate;
		} else if (angle < 0.0)	{
			leftStickValue = -rotateToAngleRate;
			rightStickValue = rotateToAngleRate;
		}
		
		leftDrivetrain.set(leftStickValue);
		rightDrivetrain.set(rightStickValue);
		
		// Determine if the PID is finished
		if (Math.abs(Math.abs(angle) - Math.abs(imuGetYaw())) < Constants.kToleranceDegrees ) {
			if (!timing) {
				intervalTimer.start();
				timing = true;
			}		
		} else {
			intervalTimer.reset();
			timing = false;
		}
		
		// Check to see if PID has succeeded, or timed out and failed
		if (intervalTimer.hasPeriodPassed(0.5))	{
			/*frontLeftMotor.set(ControlMode.PercentOutput, 0.0);	// stop the motors
			frontRightMotor.set(ControlMode.PercentOutput, 0.0);	// stop the motors
			rearLeftMotor.set(ControlMode.PercentOutput, 0.0);
			rearRightMotor.set(ControlMode.PercentOutput, 0.0);*/
			frontLeftSpark.set(0.0);
			frontRightSpark.set(0.0);
			centerLeftSpark.set(0.0);
			centerRightSpark.set(0.0);
			rearLeftSpark.set(0.0);
			rearRightSpark.set(0.0);
			Robot.isTurning = false;
			intervalTimer.reset();
			failTimer.reset();
			turnController.reset();
			Robot.targetTurn = false;
			return 0;
		} else if (failTimer.hasPeriodPassed(timeout)) {	// the PID has failed!
			/*frontLeftMotor.set(ControlMode.PercentOutput, 0.0);	// stop the motors
			frontRightMotor.set(ControlMode.PercentOutput, 0.0);	// stop the motors
			rearLeftMotor.set(ControlMode.PercentOutput, 0.0);
			rearRightMotor.set(ControlMode.PercentOutput, 0.0);*/
			frontLeftSpark.set(0.0);
			frontRightSpark.set(0.0);
			centerLeftSpark.set(0.0);
			centerRightSpark.set(0.0);
			rearLeftSpark.set(0.0);
			rearRightSpark.set(0.0);
			Robot.isTurning = false;
			intervalTimer.reset();
			failTimer.reset();
			turnController.reset();
			Robot.targetTurn = false;
			return -1;
		}
		else	{		// the PID is not complete
			return 1;
		}
	}


	int turnToAngle(double time, double power, boolean direction){  //clockwise is true && counterclockswise is false
		if(!Robot.isDeadReckonInitialize){
			turnTimer.start();
			Robot.isDeadReckonInitialize = true;
			System.out.println("initialized!");
		}
		if(direction){
			leftDrivetrain.set(power);
			rightDrivetrain.set(-power);
			System.out.println("Turn Clockwise!");
		} else if (!direction){
			leftDrivetrain.set(-power);
			rightDrivetrain.set(power);
			System.out.println("Turn CounterClockwise!");
		} else {
			leftDrivetrain.set(0.0);
			rightDrivetrain.set(0.0);
			System.out.println("ERROR");
			return -1;
		}

		if(turnTimer.hasPeriodPassed(time)){
			leftDrivetrain.set(0.0);
			rightDrivetrain.set(0.0);
			System.out.println("Turn Complete");
			return 0;
		} else {
			return 1; //still running
		}
		
	}

	/**
	 * Pid controller for turning without a gyro
	 * Likely will not be as accurate as gyro turning
	 * 
	 * Positive angle = right turn
	 */
	int turnToEncoders(double angle)
	{
		if (!Robot.isTurning) 
		{
			initializeEncoders();	// zero the encoders
			Robot.isTurning = true;
		} 
		double error = getEncoderAngle() - angle;
		Robot.error = error;
		double output;
		if (error < 1 && error > -1)
		{
			output = 0; // 0
		}
		else if (error < 5 && error > -5)
		{
			output = 0.12; // .12
		}
		else if (error < 15 && error > -15)
		{
			output = 0.24; // .24
		}
		else
		{
			output = 0.45; // .45
		}
		leftDrivetrain.set(output);
		rightDrivetrain.set(output);

		if (error < 1 && error > -1)	{
			Robot.isTurning = false;
			rightDrivetrain.setInverted(false);
			return 0;
		} else	{
			return 1;
		}
	}

    /*
	 *  Drives straight by a specified distance
	 *  Positive distance is forward, negative reverse
	 *  
	 *  Also requires a timeout value, which is time before we give up PID control and
	 *  move to whatever next step
	 *  
	 *  Returns:
	 *  0 if not enabled
	 *  1 if enabled and turning
	 *  -1 if error
	 */
	int driveTo(double distance, double timeout)	
	{	
		if (!Robot.isDriving) 
		{
			driveController.setSetpoint(distance);
			turnYawController.setSetpoint(imuGetYaw()); // TODO imuGetYaw()
			driveController.setOutputLimits(-0.4, 0.4); //-0.8, 0.8
			rotateToAngleRate = 0; 	// This value will be updated in the pidWrite() method
			driveToDistanceRate = 0;
			initializeEncoders();	// zero the encoders
			leftStickValue = rightStickValue = 0.0;
			failTimer.start();		// the PID will fail if this timer exceeded
			Robot.isDriving = true;
			System.out.println("Finish initializing\n\n");
		} 

		// automatically turn toward target center
		// if (isCameraControlling)
		// 	turnYawController.setSetpoint(angle);
		
		// This is the final output of the PID
		driveToDistanceRate = driveController.getOutput(-getEncoderDistance(), distance);	// this uses the left encoder
		rotateToAngleRate = turnYawController.getOutput(imuGetYaw()); 	// setpoint already loaded
		double d =  getEncoderDistance();
		currentDistanceError = distance + d;
		//leftStickValue = -driveToDistanceRate - rotateToAngleRate;
		//rightStickValue = -driveToDistanceRate + rotateToAngleRate;
		leftStickValue = -driveToDistanceRate;
		rightStickValue = -driveToDistanceRate;
		leftDrivetrain.set(-leftStickValue);
		Robot.rightSpeed = rightStickValue;
		rightDrivetrain.set(rightStickValue);
		Robot.distance = d;
		Robot.error = currentDistanceError;
		
		// When we get close to the target, dynamically adjust PI terms
		if (currentDistanceError < (Constants.kToleranceDistance * 2))	{
		 	driveController.setI(Constants.kDrive_Ia);	// 0.0005
		 	driveController.setP(Constants.kDrive_Pa);	// 0.08
		}	

		if (currentDistanceError < Constants.kToleranceDistance ) 	{
			if (!timing) {
				intervalTimer.start();
				//System.out.println("Started PID timer in driveTo");
				timing = true;
			} 
		} else 	{					
			intervalTimer.reset();
			timing = false;
		}
		
		if (intervalTimer.hasPeriodPassed(1.0))	{					// Within deadband for interval time
			/*frontLeftMotor.set(ControlMode.PercentOutput, 0.0);		// stop the motors
			frontRightMotor.set(ControlMode.PercentOutput, 0.0);	// stop the motors
			rearLeftMotor.set(ControlMode.PercentOutput, 0.0);
			rearRightMotor.set(ControlMode.PercentOutput, 0.0);*/
			frontLeftSpark.set(0.0);
			frontRightSpark.set(0.0);
			centerLeftSpark.set(0.0);
			centerRightSpark.set(0.0);
			rearLeftSpark.set(0.0);
			rearRightSpark.set(0.0);
			Robot.isDriving = false;
			intervalTimer.reset();
			failTimer.reset();
			driveController.reset();
			turnController.reset();
			rightDrivetrain.setInverted(false);
			System.out.println("Pid Finished Correctly\n\n");
			return 0;	// PID is complete (successful)
		} else if (failTimer.hasPeriodPassed(timeout)) 	{			// the PID has failed!
			/*frontLeftMotor.set(ControlMode.PercentOutput, 0.0);		// stop the motors
			frontRightMotor.set(ControlMode.PercentOutput, 0.0);	// stop the motors
			rearLeftMotor.set(ControlMode.PercentOutput, 0.0);
			rearRightMotor.set(ControlMode.PercentOutput, 0.0);*/
			frontLeftSpark.set(0.0);
			frontRightSpark.set(0.0);
			centerLeftSpark.set(0.0);
			centerRightSpark.set(0.0);
			rearLeftSpark.set(0.0);
			rearRightSpark.set(0.0);
			Robot.isDriving = false;
			intervalTimer.reset();
			failTimer.reset();
			driveController.reset();
			turnController.reset();
			rightDrivetrain.setInverted(false);
			System.out.println("Pid Timout\n\n");
			return -1;	// PID has failed (timeout)
		} else	{		// the PID is not complete
			return 1;
		}
	}
}
