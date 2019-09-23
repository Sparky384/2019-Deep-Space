/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Set;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */

/*
CAMERA NOTE
The usb port at the top of the rio is device 1. Whichever camera is plugged in 
here is device 1 (indexed at 0) and should be saved in the cam1 variable.
As of now (3/13) the bottom camera should be cam1 (device index 0).
Once you plug in the cameras correctly and have tested them, do not unplug them
unless you HAVE to, this will lessen confusion as to which camera goes where.

The "top" is the side closest to the edge (the outermost port), farthest from 
the gyro port
*/
public class Robot extends IterativeRobot {
	private UsbCamera cam1; //bottom camera, see CAMERA NOTE in comment at line 40
	private UsbCamera cam2; //top camera, see CAMERA NOTE in comment at line 40
	private CameraServer server;
	private CameraServer server2;
	private Drivetrain drive;
	private Intake in;
	private Wrist wrist;
	private Climber climb;
	private ArrayList<Double> angles;
	private Thread t1;
	private Timer climberTimer;
	private Timer driveTimer;
	private Timer endgameTimer;
	private NetworkTableEntry cameraEntry;
	
	boolean[] wristLocks;		// Boolean array indicates which wrist position is active
	boolean running; // TODO
	boolean auto;
	boolean climberMaxSpeed;
	boolean startClimberTimer;
	boolean cameraButtonLock;
	boolean piComm;
	boolean forceDown;
	boolean elevating;	// Climbing in level 2 only
	boolean lvl2Lock;	// True when in level 2 climb
	boolean endgameTimerTicking;
	boolean lock;
	int wristSetpoint;	// the angle of the wrist in encoder counts
	int currentCamera; 
	Lvl2 state;

	public volatile double offAngle;	// for auto target finding

	public static boolean targetTurn = false;
	public static boolean isDriving = false;
	public static boolean isTurning = false;
	public static double rightSpeed = 0.0; 
	public static double leftSpeed = 0.0;
	public static double distance = 0.0;
	public static double error = 0.0;
	static boolean isWristControlEnabled = false;	// wrist is "wristing"
	static boolean isWristMoving = false;			// true if wrist in motion
	static boolean isWristGoingFullUp = false;
	
	public Robot() {
		drive = new Drivetrain();
		in = new Intake();
		climb = new Climber();
		wrist = new Wrist();
		angles = new ArrayList<>();
		wristLocks = new boolean[5];	// set array to 5 elements
		climberTimer = new Timer();
		driveTimer = new Timer();
		endgameTimer = new Timer();
		t1 = new Thread(new ClientThread());

	}
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// see CAMERA NOTE in comment above Robot class
		server = CameraServer.getInstance();
		cam1 = server.startAutomaticCapture(0);
		server2 = CameraServer.getInstance();
		cam2 = server2.startAutomaticCapture(1);
		cameraEntry = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
		cameraEntry.setString(cam2.getName()); 
		
		piComm = false;
		startClimberTimer = false;
		climberMaxSpeed = false;
		forceDown = false;
		elevating = false;	// Climbing in level 2 only
		lvl2Lock = false;
		cameraButtonLock = false;
		endgameTimerTicking = false;

		wristSetpoint = 0;
		currentCamera = 2;
		state = Lvl2.IntakeDown;
		lock = false;

		//t1.start();
	}
	
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		auto = true;
		wrist.initializeEncoders();
	}
	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		driverControl(); //autonomous is driver controlled, refers to one method to avoid confusion
	}

	public void teleopInit()
	{
		auto = false;
	}
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() 
	{
		driverControl(); //auton and teleop refer to one method to keep code clean and to avoid confusion
	}
	public void teleopDisable()
	{
		running = false;
	}

	public void driverControl()
	{
		//================PILOT STICK================
		// pilot stick driving
		if (Input.getButtonHeld(Constants.PILOT, ButtonMap.trigger))
			drive.arcadeDrive(-Input.getAxis(Constants.PILOT, Constants.YAXIS), 
								Input.getAxis(Constants.PILOT, Constants.XAXIS));
		else
			drive.arcadeDrive(0, 0);

		// drive to cargo ship
		/*if (Input.getButtonPressed(Constants.PILOT, ButtonMap.toTarget))
			targetTurn = true;
		if (Input.getButtonHeld(Constants.PILOT, ButtonMap.toTarget))
			toTarget();
		if (Input.getButtonReleased(Constants.PILOT, ButtonMap.toTarget))
			targetTurn = false;*/
		
		// code to toggle cameras
		// for camera switching to work on the driver's station, the CameraServer Stream Viewer
		// on the driver station has to have the property "Selected Camera Path" equal to
		// "CameraSelection". We spent quite a bit of time trying to figure this out.

		if (Input.getButtonHeld(Constants.PILOT, ButtonMap.trigger) &&
		    Input.getButtonHeld(Constants.PILOT, ButtonMap.boost))
		{
			double axis = Input.getAxis(Constants.PILOT, Constants.YAXIS);
			System.out.printf("y axis val = %f", axis);
			if (axis <= -0.5)
				drive.boostForward();
			else if (axis >= 0.5)
				drive.boostBackward();
			else 
				drive.stopBoost();
		}
		else
			drive.stopBoost();

		if(Input.getButtonPressed(Constants.PILOT, ButtonMap.camera) && !cameraButtonLock){	// executes once when button pressed
			cameraButtonLock = true;
			if(currentCamera == 1){
				cameraEntry.setString(cam2.getName()); //sets camera 2 on smartdashboard
				currentCamera = 2;
			} else {
				cameraEntry.setString(cam1.getName()); //sets camera 1 on smartdashboard
				currentCamera = 1;
			}
		}

		// reset the state of buttonLock so the next button press is captured
		if(Input.getButtonReleased(Constants.PILOT, ButtonMap.camera))
			cameraButtonLock = false; 

		// intake
		in.checkLock();
		if (in.getCounting()) // ball just broke photoeye, run on timer
		{
			in.isPassed();
			in.keep();
		}
		else if (Input.getButtonHeld(Constants.PILOT, ButtonMap.intake) && !in.getLock())	// intake
			in.intake();
		else if (Input.getButtonHeld(Constants.PILOT, ButtonMap.outake))				 // Outtake low
			in.outake();
		else if (Input.getButtonHeld(Constants.PILOT, ButtonMap.outakeHigh))			 // Outtake high
			in.outakeHigh();
		else
			in.stop();

		//================COPILOT STICK================
		// copilot wrist control
		// Zero out wrist encoder if wrist is at top
		if(wrist.isWristTop())
			wrist.initializeEncoders();

		// Move wrist to high position setpoint
		if(Input.getButtonPressed(Constants.COPILOT, ButtonMap.rocketCargo)){
			if (!wristLocks[0])		// high position wristlock
			{
				if(currentCamera == 2){
					currentCamera = 1;
					cameraEntry.setString(cam1.getName());
					System.out.println("Switch to 1");
				}
				unlockWrist();	// clear any existing wristlock state
				wristLocks[0] = true;
			}
			wristSetpoint = Constants.ROCKET_CARGO;
			isWristMoving = true;
			isWristGoingFullUp = false;
			forceDown = false;
		}

		// Move wrist to low position setpoint
		if(Input.getButtonPressed(Constants.COPILOT, ButtonMap.hatchIntake)){
			if (!wristLocks[1])		// low position wristlock
			{
				if(currentCamera == 1){
					currentCamera = 2;
					cameraEntry.setString(cam2.getName());
					System.out.println("Switch to 2");
				}
				unlockWrist();		// clear any existing wristlock state
				wristLocks[1] = true;
			}
			wristSetpoint = Constants.HATCH_INTAKE;
			isWristMoving = true;
			isWristGoingFullUp = false;
			forceDown = false;
		}

		// for the hatch
		if(Input.getButtonPressed(Constants.COPILOT, ButtonMap.hatchScore)){
			if (!wristLocks[2])
			{
				if(currentCamera == 1){
					currentCamera = 2;
					cameraEntry.setString(cam2.getName());
					System.out.println("Switch to 2");
				}
				unlockWrist();		// clear any existing wristlock state
				wristLocks[2] = true;
			}
			wristSetpoint = Constants.HATCH_SCORE;
			isWristMoving = true;
			isWristGoingFullUp = false;
			forceDown = false;
		}

		// Send wrist to top
		if (Input.getButtonPressed(Constants.COPILOT, ButtonMap.wristTop)) {
			if (!wristLocks[3])
			{
				if(currentCamera == 2){
					currentCamera = 1;
					cameraEntry.setString(cam1.getName());
					System.out.println("Switch to 1");
				}
				unlockWrist();		// clear any existing wristlock state
				wristLocks[3] = true;
			}
			wristSetpoint = Constants.WRIST_TOP;
			isWristMoving = true;
			isWristGoingFullUp = true;
			forceDown = false;
		}

		// Send wrist to bottom
		if (Input.getButtonPressed(Constants.COPILOT, ButtonMap.ballIntake)) {
			if (!wristLocks[4])
			{	
				if(currentCamera == 1){
					currentCamera = 2;
					cameraEntry.setString(cam2.getName());
					System.out.println("Switch to 2");
				}
				unlockWrist();		// clear any e`xisting wristlock state
				wristLocks[4] = true;
			}
			wristSetpoint = Constants.BALL_INTAKE; //Was: 253. Changed because of the new PID for the NEO motors.
			isWristMoving = true;
			isWristGoingFullUp = false;
			forceDown = false;
		}

		// Set the position control mode for each button press
		// This code dispatches the wrist motion control to 
		// either full end of travel or move to setpoint code
		if (isWristMoving) {
			if (isWristGoingFullUp) {
				wrist.goToTop();
			} else {
				wrist.moveTo(wristSetpoint, -1);		// any of the intermidiate positions
			}
		}
		else if (forceDown)
		{
			wrist.forceDown();
		}

		// Manual wrist control - use joystick axis
		if (Input.getButtonHeld(Constants.COPILOT, ButtonMap.trigger)) {
			wristSetpoint = wrist.getEncoderPosition();
			if (!wrist.isWristTop() || (Input.getAxis(Constants.COPILOT, Constants.YAXIS) < 0)) {
				wrist.setOutput(-Input.getAxis(Constants.COPILOT, Constants.YAXIS)); 
			} else {
				wrist.stop();
			}
		} else {
			if (!isWristMoving)
			{
				wrist.stop();
			}
		}

		// If joystick trigger released, make sure to stop intake
		if(Input.getButtonReleased(Constants.COPILOT, ButtonMap.trigger)){
			isWristMoving = false;
			wrist.stop();
		}

		// Bump the climber for winding 
		if (Input.getButtonHeld(Constants.COPILOT, ButtonMap.climberWind)){
			if (!startClimberTimer)  
			{
				climberTimer.reset();
				climberTimer.start();
				startClimberTimer = true;
			}
			// Speed ramp finished, running full speed now
			if (climberMaxSpeed || climberTimer.hasPeriodPassed(Constants.CLIMBER_RAMPUP))
			{
				climb.windClimb();
				climberTimer.stop();
				climberMaxSpeed = true;
			}
			// Get ramp value
			else  
				climb.variableClimb(climberTimer.get() / Constants.CLIMBER_RAMPUP);
		}

		// Climber control for Level 3
		if(Input.getButtonHeld(Constants.COPILOT, ButtonMap.lvl3climb) && !auto)
		{
			stopClimberFromMoving();
			// first time in, initialize climber ramp logic
			if (!startClimberTimer)  
			{
				wrist.setBrakeMode(true);
				climberTimer.reset();
				climberTimer.start();
				startClimberTimer = true;
			}
			// Speed ramp finished, running full speed now
			if (climberMaxSpeed || climberTimer.hasPeriodPassed(Constants.CLIMBER_RAMPUP))
			{
				climb.windClimb();
				climberTimer.stop();
				climberMaxSpeed = true;
			}
			// Get ramp value
			else  
				climb.variableClimb(climberTimer.get() / Constants.CLIMBER_RAMPUP);
		}
		
		if(Input.getButtonReleased(Constants.COPILOT, ButtonMap.lvl3climb) || 
			Input.getButtonReleased(Constants.COPILOT, ButtonMap.climberWind))
		{
			startClimberTimer = false;
			climberMaxSpeed = false;
			climberTimer.stop();
			climb.stop();
			endgameTimerTicking = true;
			endgameTimer.reset();
			endgameTimer.start();
			isWristMoving = false;
		}

		// keep intake down whist robot is falling
		if (endgameTimerTicking)
		{
			if (!endgameTimer.hasPeriodPassed(30))
				stopClimberFromMoving();
			else
			{
				endgameTimer.stop();
				endgameTimerTicking = false;
			}
		}

		if (Input.getButtonHeld(Constants.PILOT, ButtonMap.sandstormDrive))
		{

		}

		// Level 2 climb
		// *TODO* fix code so that the wristTimer is reset cleanly - right now we have to 
		// restart code each time - deploying is not sufficient
		// START OF AUTOMATED LEVEL 2 - BROKEN
		// Don't use, this code does not work yet!
		/*

		// Why do we have 2 events - getButtonPressed and getButtonHeld?
		if (Input.getButtonPressed(Constants.COPILOT, ButtonMap.lvl2climb) && !auto)
		{
			if (!lvl2Lock)	// level 2 climb initialization
			{           
				wristSetpoint = 0;
				driveTimer.stop();		
				driveTimer.reset();
				climberTimer.stop();
				climberTimer.reset();
				state = Lvl2.IntakeDown;		// initial start state
				lvl2Lock = true;				// stays true until level 2 complete
				isWristMoving = false;
				elevating = false;				// Climbing in level 2 only
				System.out.println("Level 2 climb state initialization, state = " + state);
			}
		}
	
		
		if(Input.getButtonHeld(Constants.COPILOT, ButtonMap.lvl2climb) && !auto)
		{
			boolean finished = false;
			int finished2 = 1;	// The result from wrist.moveTo() - poor naming
			// Wrist initialization
			if (state == Lvl2.IntakeDown && !isWristMoving)
			{
				wristSetpoint = Constants.WRIST_LVL2;	// Establish setpoint, this is below the floor
				isWristMoving = true;					// OK, the wrist is moving
				System.out.println("Entered intake down state, state = " + state);
			}
	
			// Move wrist to low setpoint to raise robot front wheels
			if (isWristMoving && state == Lvl2.IntakeDown)
			{
				finished2 = wrist.moveTo(wristSetpoint, 2.5);	// The result from wrist.moveTo() - poor naming
				System.out.println("Moving intake down to elevate robot = " + state);
			}

			// Finish wrist low state, start drive state
			if (state == Lvl2.IntakeDown && finished2 < 1)
			{
				isWristMoving = false;
				state = Lvl2.DriveToHab;
				System.out.println("Finish wrist low state, drive state, finish2 = " + finished2);
				System.out.println("Finish wrist low state, drive state, state = " + state);
			}
			
			// In drive state
			if (state == Lvl2.DriveToHab)
			{
				int com = drive.driveTo(9.0, 1.0);
				System.out.println("Driving");
				if (com < 1) {	// Test for PID failure/complete
					state = Lvl2.StoreIntake;
					System.out.println("Driving completed/failed, start wrist store state, state = " + state);
				}
			}

			// Start the wrist up to store position state
			if (state == Lvl2.StoreIntake && !isWristMoving)
			{
				isWristMoving = true;
				System.out.println("Entered wrist store (up) state, state = " + state);
			}
			
			// Move the wrist up to store position
			if (state == Lvl2.StoreIntake && isWristMoving)
			{
				finished = wrist.goToTop();
				System.out.println("Moving wrist to store posn, state = " + state);
				if (finished)
				{
					isWristMoving = false;
					state = Lvl2.Climb;
					System.out.println("Wrist store complete, start climber state, state = " + state);
				}
			}

			// Start the climber state - this could all go in the if block above
			if (state == Lvl2.Climb && !elevating)
			{
				climberTimer.reset();
				climberTimer.start();
				elevating = true;		// Climbing in level 2 only
			}
			
			// Climb
			if (elevating && state == Lvl2.Climb)
				climb.windClimb();		// This is the full speed climb with no accel ramp
				System.out.println("Climbing, state = " + state);

			if (state == Lvl2.Climb && climberTimer.hasPeriodPassed(0.85))
			{
				state = Lvl2.FinishDrive;
				driveTimer.reset();
				driveTimer.start();
				climb.stop();
				System.out.println("Entered finish state, state = " + state);
			}
			// Do the finish drive
			if (state == Lvl2.FinishDrive)
			{
				//climb.windClimb();
				drive.arcadeDrive(0.65, 0.25);
				System.out.println("Doing the finish drive, state = " + state);
			}

			// Check for drive finished
			if (state == Lvl2.FinishDrive && driveTimer.hasPeriodPassed(1.0)) {
				state = Lvl2.Stop;
				System.out.println("Entered stop state, state = " + state);
			}

			// cleanup, level 2 routine finished
			if (state == Lvl2.Stop)
			{
				climb.stop();
				drive.arcadeDrive(0, 0);
				wrist.stop();
				System.out.println("Executing stop state, state = " + state);
			}
		}

		if (Input.getButtonReleased(Constants.COPILOT, ButtonMap.lvl2climb)) {
			climb.stop();
			drive.arcadeDrive(0, 0);
			wrist.stop();
			wristSetpoint = 0;
			driveTimer.stop();
			driveTimer.reset();
			climberTimer.stop();
			climberTimer.reset();
			lvl2Lock = false;
			isWristMoving = false;
			elevating = false;
			startClimberTimer = false;
			climberMaxSpeed = false;
		}
		*/ 
		// END OF AUTOMATED LEVEL 2 - BROKEN

		// Unwind the climber - NOT FOR MATCH USE!!
		if(Input.getButtonHeld(Constants.COPILOT, ButtonMap.climberUnwind) && !auto) {
			climb.unwindClimb();
		}
		
		// Stop the unwind
		if(Input.getButtonReleased(Constants.COPILOT, ButtonMap.climberUnwind) && !auto) {
			climb.stop();
			wrist.stop();
		}
		dashboardOutput();
	}

	private void dashboardOutput() {
		SmartDashboard.putBoolean("ball in", in.isBallPresent());
		SmartDashboard.putBoolean("wrist top", wrist.isWristTop());
		SmartDashboard.putNumber("wrist encoder", wrist.getEncoderPosition());	
	}

	private void stopClimberFromMoving()
	{
		wrist.setOutput(0.0);
		wristSetpoint = wrist.getEncoderPosition();
	}

	private void unlockWrist(){
		for(int i=0; i<wristLocks.length; i++){
			wristLocks[i] = false;
		}
	}

	// sandstorm stuff
	private void toTarget()
	{
		// git em boi
		// if (targetTurn)
		// 	drive.turnTo(offAngle, 3.0);
		// if (!targetTurn)
		// 	drive.driveTo(drive.getEncoderDistance() + 7, 3.0, true, offAngle);
	}

	// thread to communicate with pi
	private class ClientThread implements Runnable
	{
		BufferedReader br;
		BufferedWriter bw;
		String ipaddress = "10.3.84.50";
		int missedData = 0;
		int port = 50003;
		Socket sock;
		
		@Override
		public void run() 
		{
			while (true)
			{
				while(createSocket() == 1);
				offAngle = 0;
				running = true;	
				System.out.println("connected\n");
				while(running)
				{
					try {
						String charCode = br.readLine();
						double angle = Double.parseDouble(charCode);
						// calculate average off angle
						if (angle != Constants.ERROR_VALUE)
						{
							System.out.println("good data");
							if (angles.size() != Constants.MAX_SIZE)
							{
								angles.add(angle);
								for (Double d : angles)
									offAngle += d;
								offAngle /= angles.size();
								System.out.printf("Angle 1 = %f\n", offAngle);
							}
							else
							{
								if (Math.abs(angle) < offAngle + Constants.ANGLE_THRESHOLD &&
										Math.abs(angle) > offAngle - Constants.ANGLE_THRESHOLD)
								{
									angles.remove(0);
									angles.add(angle);
									offAngle = 0;
									for (Double d : angles)
										offAngle += d;
									offAngle /= Constants.MAX_SIZE;
								}
								System.out.printf("Angle 2 = %f\n", offAngle);
							}
						}
						else
							missedData++;

						if (missedData == Constants.MAX_MISSED_DATA)
						{
							System.out.println("Clearing data");
							missedData = 0;
							angles.clear();
							offAngle = 0;
						}
						// check if pi is still communicating
						bw.write("message");
						bw.flush();
					} catch (IOException e) {
						running = false;
						try {
							sock.close();
						} catch (IOException e1) {
							// do nothing
						}
					} catch (NullPointerException e) {
						// do nothing
					}
				}
			}
		}
		
		private int createSocket()
		{
			piComm = false;
			//System.out.printf("Connect to %d on %s\n", port, ipaddress);
			try {
				InetAddress addr;
				sock = new Socket(ipaddress, port);
				addr = sock.getInetAddress();
				SmartDashboard.putString("Addr", addr.toString());
				br = new BufferedReader(new InputStreamReader(sock.getInputStream()));
				bw = new BufferedWriter(new OutputStreamWriter(sock.getOutputStream()));
				piComm = true;
				return 0;
			} catch (IOException e) {
				return 1;
			}
		}
	}
	
}
