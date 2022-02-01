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
    
      //Robot.m_shooter.shooterMotor.set(0.5);
      Robot.m_shooter.setShooterReference(4000);
      SmartDashboard.putNumber("Shooter Velocity", Robot.m_shooter.shooterMotor.getEncoder().getVelocity());
      if(Robot.m_shooter.shooterMotor.getEncoder().getVelocity() > 3000){
        Robot.m_feeder.feederMotor.set(0.5);

      }
      else {
        Robot.m_feeder.feederMotor.set(0);
      }
      
    
  }

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  public void end() {
    //SmartDashboard.putNumber("ShooterEnd",-10);
    Robot.m_shooter.shooterMotor.set(0);
    Robot.m_feeder.feederMotor.set(0);
    Feeder.setBall1Type(0);
    Feeder.setBall2Type(0);
    for (var i = 0; i < Robot.m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      Robot.m_ledBuffer.setRGB(i, 0, 255, 0);
    }
  }

  public void interrupted() {
    Robot.m_shooter.shooterMotor.set(0);
    Robot.m_feeder.feederMotor.set(0);
    //SmartDashboard.putNumber("ShooterInt",-10);
    Feeder.setBall1Type(0);
    Feeder.setBall2Type(0);
    for (var i = 0; i < Robot.m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      Robot.m_ledBuffer.setRGB(i, 0, 255, 0);
    }
  }

}