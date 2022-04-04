package frc.robot.commands.shooter;

// import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.CANSparkMax.IdleMode;

import java.sql.Time;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj.Timer;


public class RunShooterHighWithLogic extends Command {
  private boolean ourBall;
  private double shooterVelocity;
  private boolean atSpeed;
  private Timer timer;


  public RunShooterHighWithLogic() {
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);
    atSpeed = false;
    timer.reset();
    timer.start();
    Robot.m_shooter.setHighShot();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {

      shooterVelocity = Robot.m_shooter.shooterMotor.getEncoder().getVelocity();

      Robot.m_shooter.setShooterReference(RobotMap.shooterVelocitySetpointHigh);
      SmartDashboard.putBoolean("Shooting", true);
      SmartDashboard.putNumber("Shooter Velocity", Robot.m_shooter.shooterMotor.getEncoder().getVelocity());
      if(Robot.m_shooter.shooterMotor.getEncoder().getVelocity() > RobotMap.shooterVelocityThresholdHigh+150
      || timer.get() > 2){
        Robot.m_feeder.feederMotor.set(RobotMap.feederShootSpeed);
        SmartDashboard.putBoolean("Feeder", true);
      }
      else {
        Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
        SmartDashboard.putBoolean("Feeder", false);
      }
     
    
  }

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  public void end() {
    //SmartDashboard.putNumber("ShooterEnd",-10);
    Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);
    Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
    RobotMap.oneBall = false;
    RobotMap.twoBall = false;
    Feeder.setBall1Type(RobotMap.noBall);
    Feeder.setBall2Type(RobotMap.noBall);

    atSpeed = false;
    for (var i = 0; i < Robot.m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      Robot.m_ledBuffer.setRGB(i, RobotMap.defaultRValue, RobotMap.defaultGValue, RobotMap.defaultBValue);
    }
  }

  public void interrupted() {
    Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);
    Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
    //SmartDashboard.putNumber("ShooterInt",-10);
    Feeder.setBall1Type(RobotMap.noBall);
    Feeder.setBall2Type(RobotMap.noBall);
    RobotMap.oneBall = false;
    RobotMap.twoBall = false;
    atSpeed = false;

    for (var i = 0; i < Robot.m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      Robot.m_ledBuffer.setRGB(i, RobotMap.defaultRValue, RobotMap.defaultGValue, RobotMap.defaultBValue);
    }
  }

}