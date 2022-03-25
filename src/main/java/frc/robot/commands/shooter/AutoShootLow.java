package frc.robot.commands.shooter;

// import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;


public class AutoShootLow extends Command {
  public static Timer timer;
	Timer timer2;
  private boolean isFinished = false;
  boolean atSpeed;
  double time;

  public AutoShootLow(double time) {
    timer = new Timer();
		timer2 = new Timer();
    this.time = time;
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);
    timer.reset();
		timer2.reset();
		timer.start();
		timer2.start();
    atSpeed = false;
    Robot.m_shooter.shooting = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    // Robot.m_shooter.motorBrake.set(DoubleSolenoid.Value.kReverse);
    if (timer2.get() < time) {
      Robot.m_shooter.setShooterReference(RobotMap.shooterVelocitySetpointOurs);
      SmartDashboard.putNumber("Shooter Velocity", Robot.m_shooter.shooterMotor.getEncoder().getVelocity());
      if(Robot.m_shooter.shooterMotor.getEncoder().getVelocity() > RobotMap.shooterVelocityThreshold){
        Robot.m_feeder.feederMotor.set(RobotMap.feederShootSpeed);
        //SmartDashboard.putNumber("AutoShootFeeder", 1);
        
        // if (atSpeed == false){
        //   atSpeed = true;
        // }
      }
      else {
        Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
        //SmartDashboard.putNumber("AutoShootFeeder", 0);
        // if (atSpeed == true){
        //   Robot.m_feeder.setBall1Type(Robot.m_feeder.checkBall2());
        //   Robot.m_feeder.setBall2Type(RobotMap.noBall);
        // }
      }
    }
      
    else{
      Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);
      Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
      isFinished = true;

    }  
       //}
    
    
  }

  // Returns true when the command should end.
  public boolean isFinished() {
    return isFinished;
  }

  // Called once the command ends or is interrupted.
  public void end() {
    Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);
    Robot.m_shooter.shooting = false;
  }

  public void interrupted() {
    Robot.m_shooter.shooterMotor.set(RobotMap.turnOffShooterMotor);
    Robot.m_shooter.shooting = false;
  }

}