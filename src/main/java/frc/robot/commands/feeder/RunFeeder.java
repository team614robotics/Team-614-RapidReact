package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
//import frc.robot.subsystems.feeder.Feeder;

/**
 *
 */
public class RunFeeder extends Command {
    double speed;
	public RunFeeder(double speed) {
        requires(Robot.m_feeder);
        this.speed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
		// Robot.m_intake.setDoubleSolenoidA(Robot.m_intake.pistonIn);
		// Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonOut);
        // Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonOut);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.m_feeder.feederMotor.set(speed);
		// Robot.m_serializer.serializerMotorA.set(-0.35);
		// Robot.m_serializer.serializerMotorB.set(0.2);
		//Robot.m_feeder.changeCounterBasic();
	    
		// Robot.m_serializer.serializerMotorA.set(0);
		// Robot.m_serializer.serializerMotorB.set(0);
		// Robot.m_feeder.feederMotor.set(0);
		
	}

	// Make this return true when this Command no lo
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);//j should this be .turnOffFeederMotor
		// Robot.m_serializer.serializerMotorA.set(0);                j should we just have 1 .turnOffMotor they all = 0.0
		// Robot.m_serializer.serializerMotorB.set(0);
		// Robot.m_feeder.feederMotor.set(0);
		// Robot.m_intake.setDoubleSolenoidA(Robot.m_intake.pistonIn);
		// Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonOut);
        // Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonIn);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
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