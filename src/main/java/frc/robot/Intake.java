
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class Intake {
	private WPI_TalonSRX bottomIntakeMotor;
	private WPI_TalonSRX topIntakeMotor;
	private DigitalInput isBallIn;	// green
	private Timer ballTimer;	// PID interval timer
	private boolean counting;
	private boolean lock;

	public Intake() {
		bottomIntakeMotor = new WPI_TalonSRX(TalonPort.intakeBottom);
		topIntakeMotor = new WPI_TalonSRX(TalonPort.intakeTop);
		ballTimer = new Timer();
		isBallIn = new DigitalInput(SensorPorts.GREEN);

		counting = false;
		lock = false;

		topIntakeMotor.setInverted(false);
	}

	// run intake in when ball first breaks the photoeye
	public void keep()
	{
		bottomIntakeMotor.set(0.55);
		topIntakeMotor.set(-0.55);
	}

	// allow driver to run intake if photoeye not tripped
	public void checkLock()
	{
		if (!isBallPresent())
			lock = false;
	}

	// both are positive
	public void intake() {
		if (isBallPresent() && !counting)
		{
			ballTimer.reset();
			ballTimer.start();
			counting = true;
			lock = true;
		}
		else if (!isBallPresent()){
			bottomIntakeMotor.set(0.35);
			topIntakeMotor.set(-0.40);
		}
	}
	
	// both are negative
	public void outake() {
		counting = false;
		lock = false;
		ballTimer.stop();
		bottomIntakeMotor.set(-0.25);
		topIntakeMotor.set(0.6);
	}
	
	public void outakeHigh() {
		counting = false;
		lock = false;
		ballTimer.stop();
		bottomIntakeMotor.set(-0.6);
		topIntakeMotor.set(0.7);
	}

	public void stop()
	{
		bottomIntakeMotor.set(0.0);
		topIntakeMotor.set(0.0);
	}
	
	public boolean isBallPresent() {
		return isBallIn.get();
	}

	// check if 1 second keep timer has passed
	public boolean isPassed()
	{
		if (ballTimer.hasPeriodPassed(1) || !isBallPresent())
		{
			counting = false;
			if (!isBallPresent())
				lock = true;
			ballTimer.stop();
			return true;
		}
		else{
			return false;
		}
	}

	public boolean getLock()
	{
		return lock;
	}

	public boolean getCounting()
	{
		return counting;
	}
}