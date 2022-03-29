package frc.robot.commands.chassis;

import frc.robot.OI;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import java.sql.Time;

import javax.lang.model.util.ElementScanner6;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;

public class AutoArcadeEnocderDrive extends Command {

    double feet;
    double encoder;
    double speed;
    boolean finish;
    boolean move;
    boolean start=false;
    private Timer timer;
    private double time;
    
    public AutoArcadeEnocderDrive(double inch, boolean forward, double speed, double time) {// #endregion, boolean forward) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.m_drivetrain);
        encoder = RobotMap.ticksPerInch * inch;
        move = forward;
        this.speed = speed;
        this.time = time;
        timer = new Timer();

    }
    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        SmartDashboard.putNumber("leftMotor ", Robot.m_drivetrain.leftMotorA.getEncoder().getPosition());
        SmartDashboard.putNumber("encoder ", encoder);


        if(timer.get()>time)
        {
            finish = true;
        }
        if(!start)
        {
            Robot.m_drivetrain.leftMotorA.getEncoder().setPosition(0);
            Robot.m_drivetrain.leftMotorB.getEncoder().setPosition(0);
            Robot.m_drivetrain.rightMotorA.getEncoder().setPosition(0);
            Robot.m_drivetrain.rightMotorB.getEncoder().setPosition(0);
            timer.stop();
            timer.reset();
            timer.start();
          start = true;
        }
        if (Math.abs(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition()) < RobotMap.driveProp * encoder) {
            if (move) {
                Robot.m_drivetrain.arcadeDrive(speed, RobotMap.autoRotateValue);
            } else {
                Robot.m_drivetrain.arcadeDrive(-1 * speed, RobotMap.autoRotateValue);
            }
            // if (forward) {
            // Robot.m_drivetrain.arcadeDrive(((Robot.m_drivetrain.leftMotorA.getEncoder().getPosition()-encoder)/encoder)+.5,
            // RobotMap.autoRotateValue);
            // } else {
            // Robot.m_drivetrain.arcadeDrive(-1*(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition()-encoder)/encoder,
            // RobotMap.autoRotateValue);
            // }

        } else {
            if (Math.abs(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition()) < encoder) {
                if (move) {
                    Robot.m_drivetrain.arcadeDrive(RobotMap.driveSlow, RobotMap.autoRotateValue);
                } else {
                    Robot.m_drivetrain.arcadeDrive(-RobotMap.driveSlow, RobotMap.autoRotateValue);
                }



            } else {
                finish = true;
                SmartDashboard.putBoolean("finished", finish);
            }
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
        SmartDashboard.putNumber("Angle ", 0 /*Robot.m_navX.getAngle()*/);
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
