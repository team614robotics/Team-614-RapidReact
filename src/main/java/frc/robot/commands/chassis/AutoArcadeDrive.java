package frc.robot.commands.chassis;

import frc.robot.OI;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AutoArcadeDrive extends Command {
	
    
    public AutoArcadeDrive() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.m_drivetrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.m_drivetrain.arcadeDrive(.8 * (0.5 < 0 ? -Math.pow(0.5, 2) : Math.pow(0.5, 2)), .8 * (0 < 0 ? Math.pow(0, 2) : -Math.pow(0, 2)));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
