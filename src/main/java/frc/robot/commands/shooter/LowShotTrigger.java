package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.feeder.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.commands.feeder.*;
//import frc.robot.commands.feeder.GetColor;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
//import frc.robot.subsystems.feeder.Feeder;
import frc.robot.commands.feeder.RunFeeder;
import frc.robot.commands.shooter.AccelerateFlywheel;
import edu.wpi.first.wpilibj.Timer;


public class LowShotTrigger extends Command{

    private double shooterVelocity;
    private boolean atSpeed;
    boolean stupid;
    private int speed;
	Timer timer;

	public LowShotTrigger(int speed) 
    {
        requires(Robot.m_shooter);
        this.speed = speed;
		timer = new Timer();
    }

	// Called just before this Command runs the first time
	protected void initialize() 
    {
        Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);
        atSpeed = false;	
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() 
    {

      if (OI.driverController.getLeftTriggerAxis()<.3){
        Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
        Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);

      }
		// if (OI.driverController.getLeftTriggerAxis()>0.3 && OI.driverController.getLeftTriggerAxis()<0.7){
    //         timer.start();
    //         if(timer.get() < RobotMap.feederTime)
    //         {
    //           Robot.m_feeder.feederMotor.set(RobotMap.reverseFeederSpeed);
    //         }
    //         else{
    //             Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
    //         }
    //         Robot.m_shooter.setShooterReference(speed);
    //         stupid = false;
    //  } 
     if(OI.driverController.getLeftTriggerAxis()>0.7)
     {
            shooterVelocity = Robot.m_shooter.shooterMotor.getEncoder().getVelocity();
           Robot.m_shooter.setShooterReference(RobotMap.shooterVelocitySetpointOurs);
           SmartDashboard.putNumber("Shooter Velocity",-99999);
           if(Robot.m_shooter.shooterMotor.getEncoder().getVelocity() > RobotMap.shooterVelocityThreshold)
           {
             Robot.m_feeder.feederMotor.set(RobotMap.feederShootSpeed);
             if (atSpeed == false)
             {
               atSpeed = true;
             }
           }
           else {
             Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
             if (atSpeed == true){
               atSpeed = false;
             }
           }
        }
	   
		
	} 
		// Make this return true when this Command no lo
		protected boolean isFinished() {
			return false;
		}
				
	protected void end() 
    {
		Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);
        timer.stop();
        timer.reset();		
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() 
    {
		Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);
        atSpeed = false;
        timer.stop();
        timer.reset();		
    }
}
