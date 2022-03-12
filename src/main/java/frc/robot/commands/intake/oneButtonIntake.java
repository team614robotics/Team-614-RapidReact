package frc.robot.commands.intake;

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

/**
 *
 */
public class oneButtonIntake extends Command 
{
    double speed;
    boolean stupid;

	public oneButtonIntake(double speed) 
    {
        requires(Robot.m_intake);
        this.speed = speed;
        stupid = true;
	}

	// Called just before this Command runs the first time
	protected void initialize() 
    {
        Robot.m_intake.intakeMotor.set(RobotMap.turnOffIntakeMotor);	
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() 
    {
		if (OI.driverController.getLeftTriggerAxis()>0.5){
            Robot.m_intake.intakeSolenoidA.set(DoubleSolenoid.Value.kForward);
            Robot.m_intake.intakeSolenoidB.set(DoubleSolenoid.Value.kForward);
            RobotMap.isDumb = false; 
            Robot.m_intake.intakeMotor.set(RobotMap.intakeSpeed);
            stupid = false;
        } else {
            //if(!RobotMap.isDumb){
            Robot.m_intake.intakeMotor.set(RobotMap.turnOffIntakeMotor);
                //Robot.m_intake.toggleIntake();
            if (stupid == false){
                Robot.m_intake.intakeSolenoidA.set(DoubleSolenoid.Value.kReverse);
                Robot.m_intake.intakeSolenoidB.set(DoubleSolenoid.Value.kReverse);
                RobotMap.isDumb = true;
                stupid = true;
            } else {

            }

            //}
        }
        //Robot.m_intake.toggleIntake();
	   
		
	} 
		// Make this return true when this Command no lo
		protected boolean isFinished() {
			return false;
		}
				
	protected void end() 
    {
		Robot.m_intake.intakeMotor.set(RobotMap.turnOffIntakeMotor);
		//Robot.m_intake.toggleIntake();
        if(!RobotMap.isDumb){
            Robot.m_intake.toggleIntake();
        }
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() 
    {
		Robot.m_intake.intakeMotor.set(RobotMap.turnOffIntakeMotor);
		//Robot.m_intake.toggleIntake();
        if(!RobotMap.isDumb){
            Robot.m_intake.toggleIntake();
        }
    }
}