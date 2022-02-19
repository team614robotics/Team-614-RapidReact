package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.feeder.*;
import frc.robot.commands.feeder.*;
import frc.robot.commands.feeder.GetColor;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
//import frc.robot.subsystems.feeder.Feeder;
import frc.robot.commands.feeder.RunFeeder;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 */
public class AccelerateFlywheel extends Command {
	public AccelerateFlywheel() {
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);


	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

	    Robot.m_shooter.setShooterReference(RobotMap.shooterVelocitySetpointOurs);


			
	} 
		
			
	

	// Make this return true when this Command no lo
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
        Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);

		
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
        Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);

    }
    
    
}