package frc.robot.commands.chassis;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.SparkMaxPIDController;
// import edu.wpi.first.wpilibj.;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Command;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;

public class AutoArcadeRotate extends Command {

  public CANSparkMax autoMotor;
  private AHRS navx;
  private SparkMaxPIDController autoPIDController;
  private RelativeEncoder encoder;
  private double angleAt;
  private double angleWant;
  private Timer timer;
  boolean forward;
  boolean finish;
  boolean start;
  private double time;


  public AutoArcadeRotate(AHRS navx, double angleWant, boolean forward, double time) {
    this.navx = navx;
    this.angleWant = angleWant;
    timer = new Timer();
    timer.start();
    navx.reset();
    this.forward = !forward;
    this.time = time;

    // autoMotor = new CANSparkMax(RobotMap.shooterMotorPort, MotorType.kBrushless);
    // autoPIDController = autoMotor.getPIDController();

    // autoPIDController.setP(RobotMap.pValue);
    // autoPIDController.setI(RobotMap.iValue);
    // autoPIDController.setD(RobotMap.dValue);
    // //shooterPIDController.setIZone(RobotMap.shooterIZValue);
    // autoPIDController.setFF(RobotMap.shooterFFValue);
    // autoPIDController.setOutputRange(RobotMap.minOutput, RobotMap.maxOutput);
    // encoder = autoMotor.getEncoder();

    requires(Robot.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    // Robot.m_limelight.setLED(1);
    navx.reset();

  }

  public void setShooterReference(double setPoint) {
    autoPIDController.setReference(setPoint, com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {

    if(!start)
    {
      timer.stop();
      timer.reset();
      timer.start();
      start = !start;
    }
    if (timer.get() < time) {
      SmartDashboard.putNumber("Angle ", timer.get() /* Robot.m_navX.getAngle() */);

      if (Math.abs(Robot.m_drivetrain.getAngle()) < RobotMap.rotateProp * angleWant) {
        if (forward) {
          Robot.m_drivetrain.arcadeDrive(0, RobotMap.rotateFast);
        } else {
          Robot.m_drivetrain.arcadeDrive(0, -RobotMap.rotateFast);
        }

      } else {
        if (Math.abs(Robot.m_drivetrain.getAngle()) < angleWant) {
          if (forward) {
            Robot.m_drivetrain.arcadeDrive(0, RobotMap.rotateSlow);
          } else {
            Robot.m_drivetrain.arcadeDrive(0, -RobotMap.rotateSlow);
          }
        }
        else
        {
          finish = true;
        }
      }
    } 
    else {
      finish = true;
    }

  }

  // Returns true when the command should end.
  public boolean isFinished() {
    return finish;
  }

  // Called once the command ends or is interrupted.
  public void end() {
    Robot.m_drivetrain.arcadeDrive(0.0, 0.0);
    navx.reset();
    timer.stop();
    timer.reset();
    // Robot.m_limelight.setLED(0);
  }

  public void interrupted() {
    Robot.m_drivetrain.arcadeDrive(0.0, 0.0);
    navx.reset();
    timer.stop();
    timer.reset();
    // Robot.m_limelight.setLED(0);
  }
}