package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;

// Spark Max libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Wrist {
	//Motors
	private WPI_TalonSRX wristMotorEncoder; // wrist arm encoder, used to be wrist motor too
	private CANSparkMax wristMotor;
	private MiniPID wristController; 		// wrist position pid controller
	private double rotateToPositionRate; 	// wrist position pid output
	private double wristOutput; 			// value to wrist motor
	private Timer intervalTimer; 			// PID interval timer
	private Timer failTimer; 				// PID fails if value exceeded
	private boolean timing; 				// PID interval timer timing
	private boolean inDeadBand;				// wrist position is within error deadband
	private DigitalInput wristTop;			// white
	private boolean isInitialized;			// false if first entry into moveTo() block
	private double currentWristError;		// temporary vriable that is used to determine
											// if in deadband. This should be moved inside
											// of the moveTo() method
	
	public Wrist() {
		wristMotorEncoder = new WPI_TalonSRX(TalonPort.wristEncoder);
		wristMotorEncoder.setInverted(true);
		wristMotorEncoder.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

		wristMotor = new CANSparkMax(TalonPort.wristMotor, MotorType.kBrushless);
		wristMotor.restoreFactoryDefaults();
		wristMotor.setIdleMode(IdleMode.kBrake);
		wristMotor.setInverted(true);

		wristController = new MiniPID(Constants.kWrist_Pu, Constants.kWrist_Iu, Constants.kWrist_Du, Constants.kWrist_FFu);
		wristController.setOutputLimits(-0.37, 0.52); // -0.37, 0.52
		
		intervalTimer = new Timer();
		failTimer = new Timer();
		wristTop = new DigitalInput(SensorPorts.WHITE);

		timing = false;
		isInitialized = false;
		inDeadBand = false;
	}
	
	public boolean isMoving()
	{
		if (Math.abs(rotateToPositionRate) > 0)	{
			return true;
		} else {
			return false;
		}
	}
	
	public void initializeEncoders() {
		wristMotorEncoder.getSensorCollection().setQuadraturePosition(0, 0);
		wristMotorEncoder.setSensorPhase(false);  // runs negative otherwise, due to being mirrored
	}

	/*
	 * Get encoder position Consider making this a double, getting the position in
	 * inches Or the method below could return position
	 */
	public int getEncoderPosition() {
		return wristMotorEncoder.getSensorCollection().getQuadraturePosition();
	}

	/*
	 * moveTo for PID positioning
	 * Requires position as an int that ranges from 0 (bottom) to ~ 3200 (top) 
	 * Returns: 1 = PID not complete 0 = PID complete -1 = error
	 * timeout = -1 to ignore timeout
	 */
	public int moveTo(int position, double timeout) {
		if (!isInitialized) {
			wristController.setSetpoint(position);
			rotateToPositionRate = 0;
			failTimer.reset();
			failTimer.start();		// the PID will fail if this timer exceeded
			isInitialized = true;
		}

		// Calculate the current error
		currentWristError = Math.abs(position - getEncoderPosition());
		System.out.println("PID error: " + currentWristError);
		if (currentWristError < Constants.kWristToleranceDistance) {
			inDeadBand = true;
		} else{
			inDeadBand = false;
		}
			
		// This is the final output of the PID
		if (!inDeadBand) {
			rotateToPositionRate = wristController.getOutput(getEncoderPosition(), position);
		} else {
			rotateToPositionRate = Constants.kWrist_FFu;
		}

		wristMotor.set(rotateToPositionRate);
		
		if(position < getEncoderPosition()) {	// wrist has rotated down past setpoint
			wristController.setP(Constants.kWrist_Pu);	// PID constants for UP motion
			wristController.setI(Constants.kWrist_Iu);
			wristController.setD(Constants.kWrist_Du);
			wristController.setF(Constants.kWrist_FFu);
		} else if (position > getEncoderPosition()) {	// wrist is not yet at setpoint
			wristController.setP(Constants.kWrist_Pd);	// PID constants for DOWN motion
			wristController.setI(Constants.kWrist_Id);
			wristController.setD(Constants.kWrist_Dd);
			wristController.setF(Constants.kWrist_FFd);
		}

		// Determine if the PID is finished
		if (inDeadBand) {
			if (!timing) {
				intervalTimer.start();
				timing = true;
			}		
		} else {
			intervalTimer.reset();
			timing = false;
		}
		
		// Check to see if PID has succeeded, or timed out and failed

		// The PID has passed, error is within deadband for timeout value
		if (intervalTimer.hasPeriodPassed(0.5))	{
			wristMotorEncoder.set(ControlMode.PercentOutput, 0.0);	// stop the motors
			intervalTimer.stop();
			intervalTimer.reset();
			failTimer.stop();
			failTimer.reset();
			wristController.reset();
			return 0;
		}
		// The PID has failes, error is not within deadband for timeout value
		else if (failTimer.hasPeriodPassed(timeout) && timeout > 0)
		{
			wristMotorEncoder.set(ControlMode.PercentOutput, 0.0);	// stop the motors
			intervalTimer.stop();
			intervalTimer.reset();
			failTimer.stop();
			failTimer.reset();
			wristController.reset();
			return -1;
		} 
		// The PID is still running
		else {
			//System.out.println("PID runnning");
			return 1;
		}
	}
	
	public void stop() {
		wristMotor.set(0);
	}

	public boolean goToTop() {
		double speed; // how fast to drop
		int encoder;
		
		encoder = (getEncoderPosition());

		if (encoder > 200)	// far from top
			speed = -0.60; // -0.6
		else if (encoder > 100)
			speed = -0.48; // -0.48
		else if (encoder > 50)
			speed = -0.38; // -0.38
		else if (encoder > 20)
			speed = -0.27; // -0.27
		else	// near top
			speed = -0.22; // -0.22
		
		wristMotor.set(speed);

		if (isWristTop()) {
			stop();
			speed = 0;
			return true;
		} else {
			return false;
		}
	}

	public void forceDown()
	{
		wristMotor.set(0.15);
	}

	public boolean isWristTop()
	{
		return !wristTop.get();
	}

	public void setOutput(double output) {
		wristMotor.set(output);
	}

	public void setBrakeMode(boolean mode)
	{
		if (mode)
			wristMotor.setIdleMode(IdleMode.kBrake);
		else
			wristMotor.setIdleMode(IdleMode.kCoast);
	}
}
