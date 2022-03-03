package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;

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
public class RunIntakeBasic extends Command {
    double speed;
	Command intakeGetColor;
	Timer timer;
	boolean continueColor;
	boolean doColor;
	public RunIntakeBasic(double speed, boolean doColor) {
        requires(Robot.m_intake);
		intakeGetColor = new GetColor();
        this.speed = speed;
		timer = new Timer();
		continueColor = false;
		this.doColor = doColor;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        Robot.m_intake.intakeMotor.set(RobotMap.turnOffIntakeMotor);
		timer.reset();
		continueColor = false;
		

		//Robot.m_feeder.feederMotor.set(0);
		// Robot.m_intake.setDoubleSolenoidA(Robot.m_intake.pistonIn);
		// Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonOut);
        // Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonOut);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		if (doColor){
			intakeGetColor.start();
			Robot.m_feeder.doColor = true;
		}
		Robot.m_intake.intakeMotor.set(speed);
		continueColor = true;

			
		// Robot.m_serializer.serializerMotorA.set(-0.35);
		// Robot.m_serializer.serializerMotorB.set(0.2);
		//Robot.m_feeder.changeCounterBasic();
	} 
				
		// Robot.m_serializer.serializerMotorA.set(0);
		// Robot.m_serializer.serializerMotorB.set(0);
		// Robot.m_feeder.feederMotor.set(0);
		
	// Make this return true when this Command no lo
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_intake.intakeMotor.set(RobotMap.turnOffIntakeMotor);
		// if (doColor){
		// 	if (continueColor == true){
		// 		timer.start();
		// 		if (timer.get() > RobotMap.colorTime){
					intakeGetColor.cancel();
					Robot.m_feeder.doColor = false;
					timer.stop();
					continueColor = false;
					timer.reset();
					//Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
					//Robot.m_feeder.continueFeeder();
					OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.rumbleOff);
            		OI.driverController.setRumble(GenericHID.RumbleType.kLeftRumble, RobotMap.rumbleOff);
		// 		}
		// 	}
		// }
		//Robot.m_feeder.feederMotor.set(0);
		// Robot.m_serializer.serializerMotorA.set(0);
		// Robot.m_serializer.serializerMotorB.set(0);
		// Robot.m_feeder.feederMotor.set(0);
		// Robot.m_intake.setDoubleSolenoidA(Robot.m_intake.pistonIn);
		// Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonOut);
        // Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonIn);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.m_intake.intakeMotor.set(RobotMap.turnOffIntakeMotor);
		// if (doColor){
		// 	if (continueColor == true){
		// 		timer.start();
		// 		if (timer.get() > RobotMap.colorTime){
					intakeGetColor.cancel();
					timer.stop();
					continueColor = false;
					timer.reset();
					//Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
					//Robot.m_feeder.continueFeeder();
		// 		}
		// 	}
		// }
		OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.rumbleOff);
        OI.driverController.setRumble(GenericHID.RumbleType.kLeftRumble, RobotMap.rumbleOff);
		
		//Robot.m_feeder.feederMotor.set(0);
		// Robot.m_serializer.serializerMotorA.set(0);
		// Robot.m_serializer.serializerMotorB.set(0);
		// Robot.m_feeder.feederMotor.set(0);
		// Robot.m_intake.setDoubleSolenoidA(Robot.m_intake.pistonIn);
		// Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonOut);
        // Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonIn);
    }
    
    // protected void pistonToggle() {
    //     Robot.m_intake.toggleDoubleSolenoidA();
    //     Robot.m_intake.toggleDoubleSolenoidB();
    // }
}