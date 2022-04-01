package frc.robot.commands.chassis;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;

public class RotateToAngleBackwards extends Command {
  //private AHRS navx;
  private PIDController pid;
  private double angle;
  private Timer timer;
  private boolean isFinished;
  private double limelightAngle;

  public RotateToAngleBackwards() {
    //this.navx = navx;
    //this.angle = angle;
    timer = new Timer();
    //timer.start();

    pid = new PIDController(RobotMap.sumPValue, RobotMap.iValue, RobotMap.sumDValue);
    pid.setTolerance(0.01f);
    requires(Robot.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    //Robot.m_limelight.periodic();
    pid.reset();
    angle = Robot.m_limelight.getX();
    // Robot.m_limelight.setLED(0);
    Robot.m_limelight.setPipeline(2);
	Robot.m_limelight.setCamMode(0);
    Robot.m_limelight.setLED(0);
    timer.reset();
    timer.start();
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    SmartDashboard.putNumber("Value", pid.calculate(Robot.m_limelight.getX(), 0));
    SmartDashboard.putNumber("Target Y", Robot.m_limelight.getY());
    // double val = pid.calculate(navx.getYaw(), angle);
    Robot.m_drivetrain.arcadeDrive(0.0, pid.calculate(Robot.m_limelight.getX(), 0) < 0 ? pid.calculate(Robot.m_limelight.getX(), 0) - 0.142 : pid.calculate(Robot.m_limelight.getX(), 0) + .142);
    // if (timer.get()>1){
    //     isFinished = true;
    // }
  }

    // Returns true when the command should end.
   public boolean isFinished() {     
    return isFinished;
   }

  // Called once the command ends or is interrupted.
  public void end() {  
      Robot.m_drivetrain.arcadeDrive(0.0, 0.0);
    //   Robot.m_limelight.setLED(0);
      Robot.m_limelight.setPipeline(1);
      Robot.m_limelight.setCamMode(1);
      Robot.m_limelight.setLED(1);
  }

  public void interrupted() {
     Robot.m_drivetrain.arcadeDrive(0.0, 0.0);
    //  Robot.m_limelight.setLED(0);
     Robot.m_limelight.setPipeline(1);
     Robot.m_limelight.setCamMode(1);
     Robot.m_limelight.setLED(1);
  } 
}