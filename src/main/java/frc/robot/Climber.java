package frc.robot;
//import edu.wpi.first.wpilibj.DigitalInput;

// Spark Max libraries
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber {
//	private CANSparkMax topClimberMotor;
//	private CANSparkMax bottomClimberMotor;
	
	public Climber() {
//		topClimberMotor = new CANSparkMax(TalonPort.topClimber, MotorType.kBrushless);
//		bottomClimberMotor = new CANSparkMax(TalonPort.bottomClimber, MotorType.kBrushless);

		// Initialize the Sparks before anything else
//		topClimberMotor.restoreFactoryDefaults();
//		bottomClimberMotor.restoreFactoryDefaults();

		// Set the brake mode of the climber motors
//		topClimberMotor.setIdleMode(IdleMode.kBrake);
//		bottomClimberMotor.setIdleMode(IdleMode.kBrake);
	}

	// This is called during the speed ramp-up of the climber
	public void variableClimb(double speed) {
		speed = (speed > 1.0) ? 1.0 : speed;	// set speed to 1.0 if we go over 100%
//		topClimberMotor.set(-speed);
//		bottomClimberMotor.set(-speed);
	}

	// This is called after the speed ramp-up of the climber
	public void windClimb() {
//		topClimberMotor.set(-1.0);
//		bottomClimberMotor.set(-1.0);
	}

	public void unwindClimb() {
//		topClimberMotor.set(0.5);
//		bottomClimberMotor.set(0.5);
	}

	public void stop()
	{
//		topClimberMotor.set(0);
//		bottomClimberMotor.set(0);
	}
	
}
