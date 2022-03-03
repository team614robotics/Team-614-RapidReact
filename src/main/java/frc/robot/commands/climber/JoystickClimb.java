package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.Robot;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;

public class JoystickClimb extends Command {
    public JoystickClimb() {
        requires(Robot.m_climber);
        SmartDashboard.putNumber("ClimberJoystick", -2);
    }

    // Called when the command is initially scheduled.
    public void initialize() {
        SmartDashboard.putNumber("ClimberJoystick", -1);
        Robot.m_climber.climberMotor.set(RobotMap.initalClimberMotor);

    }

    // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        // Robot.m_climber.motorBrake.set(DoubleSolenoid.Value.kReverse);
        SmartDashboard.putNumber("ClimberJoystick", 0);
        SmartDashboard.putBoolean("UpperLimitSwitch", Robot.m_climber.limitSwitch2.get());
        SmartDashboard.putBoolean("LowerLimitSwitch", Robot.m_climber.limitSwitch1.get());
        if (OI.operatorController.getRightY() < -1 * RobotMap.joystickClimbThreshold
                || OI.operatorController.getRightY() > RobotMap.joystickClimbThreshold) {
            SmartDashboard.putNumber("ClimberJoystick", 1);
            if ((Robot.m_climber.limitSwitch2.get()&& OI.operatorController.getRightY() <=0) || (Robot.m_climber.limitSwitch1.get()&& OI.operatorController.getRightY()>=0)) {
                // if(timer.get() > .2) {
                    SmartDashboard.putNumber("ClimberJoystick", 2);
                Robot.m_climber.climberMotor.set(RobotMap.turnOffClimberMotor);
                Robot.m_climber.climberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
                 //}
            } else {
                Robot.m_climber.climberPIDController.setReference(
                        RobotMap.climberMaxVelocity * OI.operatorController.getRightY() * -1,
                        com.revrobotics.CANSparkMax.ControlType.kVelocity);
                        SmartDashboard.putNumber("ClimberJoystick", 5);
            }
        } else {
            Robot.m_climber.climberMotor.set(RobotMap.turnOffClimberMotor);
            Robot.m_climber.climberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            //SmartDashboard.putNumber("ClimberJoystick", 2);
        }
    }

    // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    public void end() {
        Robot.m_climber.climberMotor.set(RobotMap.turnOffClimberMotor);
        Robot.m_climber.climberMotor.setIdleMode(IdleMode.kBrake);
    }

    public void interrupted() {
        Robot.m_climber.climberMotor.set(RobotMap.turnOffClimberMotor);
        Robot.m_climber.climberMotor.setIdleMode(IdleMode.kBrake);
    }
}
