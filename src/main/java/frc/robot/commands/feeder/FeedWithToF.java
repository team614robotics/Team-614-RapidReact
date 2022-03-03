package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.GenericHID.*;
// import edu.wpi.first.wpilibj.GenericHID.Hand;
//import frc.robot.subsystems.feeder.Feeder;

public class FeedWithToF extends Command {
    double speed;
	public FeedWithToF() {
        requires(Robot.m_feeder);
        //this.speed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.m_feeder.timeOfFlightSensor.getRange()>RobotMap.ToFRange){
            Robot.m_feeder.feederMotor.set(RobotMap.feederSpeed);
        }
        else {
            Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
		}		
	}

	protected boolean isFinished() {
		return false;
	}

	protected void end() {
		Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
	}

	protected void interrupted() {
		Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);

    }

}