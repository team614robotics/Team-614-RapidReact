package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
//import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class MoveClimberUpwards extends Command {
   
  Timer timer; 

  public MoveClimberUpwards() {
    timer = new Timer();
    //SmartDashboard.putNumber("ClimberInit", 0);
    
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    Robot.m_climber.climberMotor.set(0);
    //SmartDashboard.putNumber("ClimberInit", 1);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    //SmartDashboard.putNumber("ClimberExecute", 1);
    if(!Robot.m_climber.limitSwitch1.get() || !Robot.m_climber.limitSwitch2.get()) {
      // if(timer.get() > .2) {
      //SmartDashboard.putNumber("ClimberExecute", 2);
      Robot.m_climber.climberMotor.set(0);
      Robot.m_climber.climberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
       //}
    } else {
      Robot.m_climber.climberMotor.set(0.5);
      //SmartDashboard.putNumber("ClimberExecute", 3);
    }
  }

    // Returns true when the command should end.
   public boolean isFinished() {
	  return !Robot.m_climber.limitSwitch1.get() || !Robot.m_climber.limitSwitch2.get() ;
   }

  // Called once the command ends or is interrupted.
  public void end() {  
    Robot.m_climber.climberMotor.set(0);
    Robot.m_climber.climberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Robot.m_climber.motorBrake.set(DoubleSolenoid.Value.kReverse);
  }

  public void interrupted() {
    Robot.m_climber.climberMotor.set(0);
    Robot.m_climber.climberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // Robot.m_climber.motorBrake.set(DoubleSolenoid.Value.kReverse);
  } 


}