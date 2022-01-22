package frc.robot.commands.shooter;

// import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;


public class AutoShoot extends Command {
  public static Timer timer;
	Timer timer2;
  private boolean isFinished = false;

  public AutoShoot() {
    timer = new Timer();
		timer2 = new Timer();
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    Robot.m_shooter.shooterMotor.set(0);
    timer.reset();
		timer2.reset();
		timer.start();
		timer2.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    // Robot.m_shooter.motorBrake.set(DoubleSolenoid.Value.kReverse);
    if (timer2.get() < 5) {
      Robot.m_shooter.shooterMotor.set(0.5);
    }
      
    else{
      Robot.m_shooter.shooterMotor.set(0);
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
    Robot.m_shooter.shooterMotor.set(0);
  }

  public void interrupted() {
    Robot.m_shooter.shooterMotor.set(0);
  }

}