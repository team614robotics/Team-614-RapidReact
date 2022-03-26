package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
//import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;


public class MoveClimberUpwards extends Command {
   
  Timer timer;
  boolean restricted;

  public MoveClimberUpwards(boolean restricted) {
    timer = new Timer();
    this.restricted = restricted;
    //SmartDashboard.putNumber("ClimberInit", 0);
    
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    Robot.m_climber.climberMotor.set(RobotMap.initalClimberMotor);
    //SmartDashboard.putNumber("ClimberInit", 1);
    timer.reset();
    timer.start();
    SmartDashboard.putNumber("Climbup", 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    SmartDashboard.putNumber("climber ", Robot.m_climber.climberMotor.getEncoder().getPosition());

    //SmartDashboard.putNumber("ClimberExecute", 1);
    if(restricted && (Robot.m_climber.climberMotor.getEncoder().getPosition() > RobotMap.MaxEncoderTick)) {
      // if(timer.get() > .2) {
      //SmartDashboard.putNumber("ClimberExecute", 2);
      Robot.m_climber.climberMotor.set(RobotMap.turnOffClimberMotor);
      SmartDashboard.putNumber("Climbup", 2);
       //}
    } else {
      //Robot.m_climber.climberPIDController.setReference(RobotMap.climberVelocitySetpoint, com.revrobotics.CANSparkMax.ControlType.kVelocity);      //SmartDashboard.putNumber("ClimberExecute", 3);
      Robot.m_climber.climberMotor.set(RobotMap.moveClimberUp);
      SmartDashboard.putNumber("Climbup", 3);
      //SmartDashboard.putNumber("ClimberUP", 1);
    }
    
  }

    // Returns true when the command should end.
   public boolean isFinished() {
	  return false;//!Robot.m_climber.limitSwitch1.get() || !Robot.m_climber.limitSwitch2.get() ;
   }

  // Called once the command ends or is interrupted.
  public void end() {  
    Robot.m_climber.climberMotor.set(RobotMap.turnOffClimberMotor);
    Robot.m_climber.climberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    SmartDashboard.putNumber("Climbup", 4);
    // Robot.m_climber.motorBrake.set(DoubleSoleno]d.Value.kReverse);
  }

  public void interrupted() {
    Robot.m_climber.climberMotor.set(RobotMap.turnOffClimberMotor);
    Robot.m_climber.climberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    SmartDashboard.putNumber("Climbup", 5);
    // Robot.m_climber.motorBrake.set(DoubleSolenoid.Value.kReverse);
  } 


}