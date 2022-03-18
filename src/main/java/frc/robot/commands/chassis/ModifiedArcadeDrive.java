package frc.robot.commands.chassis;

import frc.robot.OI;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ModifiedArcadeDrive extends Command {
	public double driveVal;
	public double rotateVal;
	public double xVal;
	public double yVal;
	public ModifiedArcadeDrive() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.m_drivetrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		xVal = OI.driverController.getRightX();
		yVal = OI.driverController.getLeftY();
		//SmartDashboard.putNumber("Driver X", xVal);
		//SmartDashboard.putNumber("Driver Y", yVal);
		//Robot.m_drivetrain.arcadeDrive(RobotMap.staticArcadeDriveValue * (OI.driverController.getLeftY() < 0 ? -Math.pow(OI.driverController.getLeftY(), 2) : Math.pow(OI.driverController.getLeftY(), 2)), 
		//RobotMap.staticArcadeDriveValue * (OI.driverController.getRightX() < 0 ? Math.pow(OI.driverController.getRightX(), 2) : -Math.pow(OI.driverController.getRightX(), 2)));
		//Robot.m_drivetrain.arcadeDrive((0.5 * OI.driverController.getLeftY()) + (0.5* (Math.pow(OI.driverController.getLeftY())), 3), 1);
		

		//driveVal = ((1-RobotMap.rampD)*yVal) + RobotMap.rampD*Math.pow(yVal, 3);//changed from 3

		//rotateVal = -1*(((1-RobotMap.rampR)*xVal) + RobotMap.rampR*Math.pow(xVal, 3));//changed from 3

		// driveVal = (0.5*yVal) + 0.5*Math.pow(yVal, 3);
		// rotateVal = -1*((0.5*xVal) + 0.5*Math.pow(xVal, 3));
		
		//SmartDashboard.putNumber("Drive Value", driveVal);
		//SmartDashboard.putNumber("Rotate Value", rotateVal);
if (OI.driverController.getLeftY()>0) {
	Robot.m_drivetrain.arcadeDrive(OI.driverController.getLeftY(), -1 * OI.driverController.getRightX());
} else {
	Robot.m_drivetrain.arcadeDrive(OI.driverController.getLeftY(), -1 * OI.driverController.getRightX());
}


// Robot.m_drivetrain.arcadeDrive(OI.driverController.getLeftY(), -1 * OI.driverController.getRightX());

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
