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


public class RunShooterHighWithLogic extends Command {
  private boolean ourBall;
  private double shooterVelocity;
  private boolean atSpeed;

  public RunShooterHighWithLogic() {
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);
    atSpeed = false;
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
    
      Robot.m_shooter.setShooterReference(RobotMap.shooterVelocitySetpointHigh);
      SmartDashboard.putNumber("Shooter Velocity", Robot.m_shooter.shooterMotor.getEncoder().getVelocity());
      if(Robot.m_shooter.shooterMotor.getEncoder().getVelocity() > RobotMap.shooterVelocityThresholdHigh){
        Robot.m_feeder.feederMotor.set(RobotMap.feederShootSpeed);
        if (atSpeed == false){
          atSpeed = true;
        }
      }
      else {
        Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
        if (atSpeed == true){
          Robot.m_feeder.setBall1Type(Robot.m_feeder.checkBall2());
          Robot.m_feeder.setBall2Type(RobotMap.noBall);
        }
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
    atSpeed = false;
    for (var i = 0; i < Robot.m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      Robot.m_ledBuffer.setRGB(i, RobotMap.defaultRValue, RobotMap.defaultGValue, RobotMap.defaultBValue);
    }
  }

}