package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.commands.feeder.*;
import frc.robot.commands.feeder.GetColor;
import edu.wpi.first.wpilibj.GenericHID.*;
//import frc.robot.subsystems.feeder.Feeder;
import frc.robot.commands.feeder.RunFeeder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 */
public class AccelerateFlywheel extends Command {
	private int speed;
	public AccelerateFlywheel(int speed) {
		this.speed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);


	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

	    Robot.m_shooter.setShooterReference(speed);
		
		boolean collisionDetected = false;
		// SmartDashboard.putNumber("check", 0);
          double curr_world_linear_accel_x = Robot.m_navX.getWorldLinearAccelX();
          double currentJerkX = curr_world_linear_accel_x - Robot.m_shooter.getAccel_x();
          Robot.m_shooter.setAccel_x(curr_world_linear_accel_x);
          double curr_world_linear_accel_y = Robot.m_navX.getWorldLinearAccelY();
          double currentJerkY = curr_world_linear_accel_y - Robot.m_shooter.getAccel_x();
          Robot.m_shooter.setAccel_y(curr_world_linear_accel_y);
          
          if ( ( Math.abs(currentJerkX) > RobotMap.collisionThreshold ) ||
               ( Math.abs(currentJerkY) > RobotMap.collisionThreshold) ) {
              collisionDetected = true;
          }
          SmartDashboard.putBoolean("CollisionDetected", collisionDetected);
		  if(collisionDetected == true)
		  {
			// SmartDashboard.putNumber("check", 1);
		  //OI.operatorController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.neutralRumble);
		  }
		  
		SmartDashboard.putNumber("Shooter Velocity", Robot.m_shooter.shooterMotor.getEncoder().getVelocity());
	
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