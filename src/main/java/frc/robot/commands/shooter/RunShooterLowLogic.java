package frc.robot.commands.shooter;

// import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.intake.*;
import edu.wpi.first.wpilibj.Timer;

public class RunShooterLowLogic extends Command {
  private boolean ourBall;
  private double shooterVelocity;
  private boolean atSpeed;
  private int speed;
  private Timer timer;

  public RunShooterLowLogic(int speed) {
    this.speed = speed;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);
    Robot.m_shooter.setLowShot();
    atSpeed = false;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    // Robot.m_shooter.motorBrake.set(DoubleSolenoid.Value.kReverse);
      if (Robot.m_feeder.checkBall1() != RobotMap.noBall){
        if (Robot.allianceColor == Robot.m_feeder.checkBall1()){
          ourBall = true;
        }
        else {
          ourBall = false;
        }
      }
      shooterVelocity = Robot.m_shooter.shooterMotor.getEncoder().getVelocity();

      
       //}
    
      //Robot.m_shooter.shooterMotor.set(0.5);
    //if (ourBall == true){
      //SmartDashboard.putNumber("Our Ball", 1);
      Robot.m_shooter.setShooterReference(speed);
      SmartDashboard.putNumber("Shooter Velocity", Robot.m_shooter.shooterMotor.getEncoder().getVelocity());
      if(Robot.m_shooter.shooterMotor.getEncoder().getVelocity() > RobotMap.shooterVelocityThreshold || timer.get() > 1){
        Robot.m_feeder.feederMotor.set(RobotMap.feederShootSpeed);
        //SmartDashboard.putNumber("Shooter Run Feeder", 1);
        if (atSpeed == false){
          atSpeed = true;
        }
      }
      else {
        Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
        //SmartDashboard.putNumber("Shooter Run Feeder", 0);
        // if (atSpeed == true){
        //   Robot.m_feeder.setBall1Type(Robot.m_feeder.checkBall2());
        //   Robot.m_feeder.setBall2Type(RobotMap.noBall);
        //   atSpeed = false;
        // }
      }
    //}
    // else if (ourBall == false){
    //   SmartDashboard.putNumber("Our Ball", 0);
    //   Robot.m_shooter.setShooterReference(RobotMap.shooterVelocitySetpointOpposing);
    //   SmartDashboard.putNumber("Shooter Velocity", Robot.m_shooter.shooterMotor.getEncoder().getVelocity());
    //   if(Robot.m_shooter.shooterMotor.getEncoder().getVelocity() > RobotMap.shooterVelocityThresholdOpposing){
    //     Robot.m_feeder.feederMotor.set(RobotMap.feederSpeed);
    //     SmartDashboard.putNumber("Shooter Run Feeder", 1);
    //     if (atSpeed == false){
    //       atSpeed = true;
    //     }
    //   }
    //   else {
    //     Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
    //     SmartDashboard.putNumber("Shooter Run Feeder", 0);
    //     if (atSpeed == true){
    //       Robot.m_feeder.setBall1Type(Robot.m_feeder.checkBall2());
    //       Robot.m_feeder.setBall2Type(RobotMap.noBall);
    //     }
    //   }
    // }
    // else{
    //   SmartDashboard.putNumber("Our Ball", -1);
    //   Robot.m_shooter.setShooterReference(RobotMap.shooterVelocitySetpointOurs);
    //   SmartDashboard.putNumber("Shooter Velocity", Robot.m_shooter.shooterMotor.getEncoder().getVelocity());
    //   if(Robot.m_shooter.shooterMotor.getEncoder().getVelocity() > RobotMap.shooterVelocityThreshold){
    //     Robot.m_feeder.feederMotor.set(RobotMap.feederSpeed);
    //     SmartDashboard.putNumber("Shooter Run Feeder", 1);
    //     if (atSpeed == false){
    //       atSpeed = true;
    //     }
    //   }
    //   else {
    //     Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
    //     SmartDashboard.putNumber("Shooter Run Feeder", 0);
    //     if (atSpeed == true){
    //       Robot.m_feeder.setBall1Type(Robot.m_feeder.checkBall2());
    //       Robot.m_feeder.setBall2Type(RobotMap.noBall);
    //     }
    //   }
    // } 
    
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

  public void interrupted() {
    Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);
    Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
    RobotMap.oneBall = false;
    RobotMap.twoBall = false;
    //SmartDashboard.putNumber("ShooterInt",-10);
    Feeder.setBall1Type(RobotMap.noBall);
    Feeder.setBall2Type(RobotMap.noBall);
    atSpeed = false;
    for (var i = 0; i < Robot.m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      Robot.m_ledBuffer.setRGB(i, RobotMap.defaultRValue, RobotMap.defaultGValue, RobotMap.defaultBValue);
    }
  }

}