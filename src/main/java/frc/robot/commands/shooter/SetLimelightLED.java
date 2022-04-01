package frc.robot.commands.shooter;

//import frc.robot.OI;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.OI;

public class SetLimelightLED extends Command {
	public SetLimelightLED() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        if (Robot.limelightLED){
            Robot.m_limelight.setPipeline(2);
		    Robot.m_limelight.setCamMode(0);
            Robot.m_limelight.setLED(0);
            Robot.limelightLED = false;
        } else {
            Robot.m_limelight.setPipeline(1);
		    Robot.m_limelight.setCamMode(1);
            Robot.m_limelight.setLED(1);
            Robot.limelightLED = true;
        }
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
