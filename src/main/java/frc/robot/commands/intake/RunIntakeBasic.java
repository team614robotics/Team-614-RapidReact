package frc.robot.commands.intake;

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
public class RunIntakeBasic extends Command {
    double speed;
	Command intakeGetColor = new GetColor();
	Timer timer;
	boolean continueColor;
	public RunIntakeBasic(double speed) {
        requires(Robot.m_intake);
        this.speed = speed;
		timer = new Timer();
		continueColor = false;
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
		if(OI.driverController.getLeftTriggerAxis() > RobotMap.triggerPressed) {
		intakeGetColor.start();
		Robot.m_intake.intakeMotor.set(speed);
		continueColor = true;

			
		// Robot.m_serializer.serializerMotorA.set(-0.35);
		// Robot.m_serializer.serializerMotorB.set(0.2);
		//Robot.m_feeder.changeCounterBasic();
	    } 
		else {
			Robot.m_intake.intakeMotor.set(RobotMap.turnOffIntakeMotor);
			if (continueColor == true){
				timer.start();
				if (timer.get() > RobotMap.colorTime){
					intakeGetColor.cancel();
					timer.stop();
					continueColor = false;
					timer.reset();
				}
			}

				
		// Robot.m_serializer.serializerMotorA.set(0);
		// Robot.m_serializer.serializerMotorB.set(0);
		// Robot.m_feeder.feederMotor.set(0);
		}
	}

	// Make this return true when this Command no lo
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_intake.intakeMotor.set(RobotMap.turnOffIntakeMotor);
		
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