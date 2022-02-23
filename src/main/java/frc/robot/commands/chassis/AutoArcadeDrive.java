package frc.robot.commands.chassis;

import frc.robot.OI;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;

public class AutoArcadeDrive extends Command {
	public static Timer timer;
	Timer timer2;
	int time;
	boolean forward;
	boolean finish;
    
    public AutoArcadeDrive(int time, boolean forward) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.m_drivetrain);

		timer = new Timer();
		timer2 = new Timer();
		this.time = time;
		this.forward = forward;
		
	
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		timer.reset();
		timer2.reset();
		timer.start();
		timer2.start();
		finish = false;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (timer2.get() < time) {
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
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return finish;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
