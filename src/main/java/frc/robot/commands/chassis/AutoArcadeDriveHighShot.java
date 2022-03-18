package frc.robot.commands.chassis;

import frc.robot.OI;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;

public class AutoArcadeDriveHighShot extends Command {
	public static Timer timer;
	Timer timer2;
	double time;
	boolean forward;
	boolean finish;
    
    public AutoArcadeDriveHighShot(boolean forward) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.m_drivetrain);


		double feet = 0;
		timer = new Timer();
		timer2 = new Timer();
		this.time = time;
		this.forward = forward;
		time = feet;
	
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		timer.reset();
		timer2.reset();
		timer.start();
		timer2.start();
		finish = false;
		Robot.m_drivetrain.leftMotorA.setIdleMode(IdleMode.kBrake);
		Robot.m_drivetrain.leftMotorB.setIdleMode(IdleMode.kBrake);
		Robot.m_drivetrain.rightMotorA.setIdleMode(IdleMode.kBrake);
		Robot.m_drivetrain.rightMotorB.setIdleMode(IdleMode.kBrake);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.m_drivetrain.leftMotorA.getEncoder().getPosition() < 0 && Robot.m_drivetrain.rightMotorA.getEncoder().getPosition() < 0) {
			if (forward){
				Robot.m_drivetrain.arcadeDrive(RobotMap.autoArcadeSpeed, RobotMap.autoRotateValue);
			}
			else {
				Robot.m_drivetrain.arcadeDrive(-1 * RobotMap.autoArcadeSpeed, RobotMap.autoRotateValue);
			}
			 
		}
		else {
			finish = true;
		}
		//.8 * (0 < 0 ? Math.pow(0, 2) : -Math.pow(0, 2)));

		// if (timer2.get() < time + 1 && timer2.get() > 1) {
		// 	if (forward){
		// 		Robot.m_drivetrain.arcadeDrive(RobotMap.autoArcadeSpeed, RobotMap.autoRotateValue);
		// 	}
		// 	else {
		// 		Robot.m_drivetrain.arcadeDrive(-1 * RobotMap.autoArcadeSpeed, RobotMap.autoRotateValue);
		// 	}
			 
		// }
		// else {
		// 	finish = true;
		// }


	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return finish;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_drivetrain.leftMotorA.setIdleMode(IdleMode.kCoast);
		Robot.m_drivetrain.leftMotorB.setIdleMode(IdleMode.kCoast);
		Robot.m_drivetrain.rightMotorA.setIdleMode(IdleMode.kCoast);
		Robot.m_drivetrain.rightMotorB.setIdleMode(IdleMode.kCoast);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.m_drivetrain.leftMotorA.setIdleMode(IdleMode.kCoast);
		Robot.m_drivetrain.leftMotorB.setIdleMode(IdleMode.kCoast);
		Robot.m_drivetrain.rightMotorA.setIdleMode(IdleMode.kCoast);
		Robot.m_drivetrain.rightMotorB.setIdleMode(IdleMode.kCoast);
	}
}
