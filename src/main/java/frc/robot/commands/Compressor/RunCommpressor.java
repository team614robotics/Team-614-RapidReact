package frc.robot.commands.Compressor;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.*;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.feeder.*;
import frc.robot.commands.feeder.*;
//import frc.robot.commands.feeder.GetColor;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
//import frc.robot.subsystems.feeder.Feeder;
import frc.robot.commands.feeder.RunFeeder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.pneumatics.*;

/**
 *
 */
public class RunCommpressor extends Command {

	public RunCommpressor() {
        requires(Robot.pneumatics);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.pneumatics.compressor.stop();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.pneumatics.compressor.start();
	} 
				

	protected boolean isFinished() {
		return false;
	}


	// Called once after isFinished returns true
	protected void end() {
		Robot.pneumatics.compressor.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	  Robot.pneumatics.compressor.stop();
    }

}