package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class MoveClimberDownwards extends Command {

  public MoveClimberDownwards() {

  }

  // Called when the command is initially scheduled.
  public void initialize() {
    Robot.m_climber.climberMotor.set(RobotMap.initalClimberMotor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    // Robot.m_climber.motorBrake.set(DoubleSolenoid.Value.kReverse);
    if(!Robot.m_climber.limitSwitch1.get()) {
      // if(timer.get() > .2) {
      
      Robot.m_climber.climberMotor.set(RobotMap.turnOffClimberMotor);
      Robot.m_climber.climberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
       //}
    } else {
      //Robot.m_climber.climberPIDController.setReference(-1 * RobotMap.climberVelocitySetpoint, com.revrobotics.CANSparkMax.ControlType.kVelocity);    
      Robot.m_climber.climberMotor.set(RobotMap.moveClimberDown); 
      SmartDashboard.putNumber("ClimberDOWN", 1);
    }
  }

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  public void end() {
    Robot.m_climber.climberMotor.set(RobotMap.turnOffClimberMotor);
    Robot.m_climber.climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void interrupted() {
    Robot.m_climber.climberMotor.set(RobotMap.turnOffClimberMotor);
    Robot.m_climber.climberMotor.setIdleMode(IdleMode.kBrake);
  }

}