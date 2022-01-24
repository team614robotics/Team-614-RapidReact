package frc.robot.commands.shooter;

// import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.feeder.*;

public class RunShooter extends Command {

  public RunShooter() {

  }

  // Called when the command is initially scheduled.
  public void initialize() {
    Robot.m_shooter.shooterMotor.set(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    // Robot.m_shooter.motorBrake.set(DoubleSolenoid.Value.kReverse);
    
      
      
       //}
    
      Robot.m_shooter.shooterMotor.set(0.5);
    
  }

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  public void end() {
    //SmartDashboard.putNumber("ShooterEnd",-10);
    Robot.m_shooter.shooterMotor.set(0);
    Feeder.setBall1Type(0);
    Feeder.setBall2Type(0);
  }

  public void interrupted() {
    Robot.m_shooter.shooterMotor.set(0);
    //SmartDashboard.putNumber("ShooterInt",-10);
    Feeder.setBall1Type(0);
    Feeder.setBall2Type(0);

  }

}