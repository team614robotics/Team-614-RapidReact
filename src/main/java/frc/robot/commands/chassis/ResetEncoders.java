package frc.robot.commands.chassis;

import frc.robot.OI;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import javax.lang.model.util.ElementScanner6;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;

public class ResetEncoders extends Command {

    double feet;
    double encoder;
    boolean finish;
    boolean move;
    boolean start = false;

    public ResetEncoders() {// #endregion, boolean forward) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.m_drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        SmartDashboard.putNumber("leftMotor ", Robot.m_drivetrain.leftMotorA.getEncoder().getPosition());
        SmartDashboard.putNumber("hi", Robot.m_drivetrain.leftMotorA.getEncoder().getPosition());

        if (!start) {
            Robot.m_navX.reset();
            Robot.m_drivetrain.leftMotorA.getEncoder().setPosition(0);
            Robot.m_drivetrain.leftMotorB.getEncoder().setPosition(0);
            Robot.m_drivetrain.rightMotorA.getEncoder().setPosition(0);
            Robot.m_drivetrain.rightMotorB.getEncoder().setPosition(0);
            start = true;
        } else {
            finish = true;
        }

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finish;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.m_drivetrain.leftMotorA.getEncoder().setPosition(0);
        Robot.m_drivetrain.leftMotorB.getEncoder().setPosition(0);
        Robot.m_drivetrain.rightMotorA.getEncoder().setPosition(0);
        Robot.m_drivetrain.rightMotorB.getEncoder().setPosition(0);
        start = false;
        SmartDashboard.putNumber("Angle ", 0 /* Robot.m_navX.getAngle() */);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.m_drivetrain.leftMotorA.getEncoder().setPosition(0);
        Robot.m_drivetrain.leftMotorB.getEncoder().setPosition(0);
        Robot.m_drivetrain.rightMotorA.getEncoder().setPosition(0);
        Robot.m_drivetrain.rightMotorB.getEncoder().setPosition(0);
        start = false;
    }
}
