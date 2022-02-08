package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import com.playingwithfusion.*;

public class Shooter extends Subsystem {
    public CANSparkMax shooterMotor;
    private SparkMaxPIDController shooterPIDController;
    private RelativeEncoder encoder;


    public Shooter() {
        shooterMotor = new CANSparkMax(RobotMap.shooterMotorPort, MotorType.kBrushless);
        shooterPIDController = shooterMotor.getPIDController();

        shooterPIDController.setP(RobotMap.shooterPValue);
        shooterPIDController.setI(RobotMap.shooterIValue);
        shooterPIDController.setD(RobotMap.shooterDValue);
        //shooterPIDController.setIZone(RobotMap.shooterIZValue);
        shooterPIDController.setFF(RobotMap.shooterFFValue);
        shooterPIDController.setOutputRange(RobotMap.minOutput, RobotMap.maxOutput);
        encoder = shooterMotor.getEncoder();

        // climberPIDController.setP(RobotMap.climberPValue);
        // climberPIDController.setI(RobotMap.climberIValue);
        // climberPIDController.setD(RobotMap.climberDValue);
        // climberPIDController.setFF(RobotMap.climberFFValue);
        // climberPIDController.setIZone(RobotMap.climberIZValue);

        // climberPIDController.setSmartMotionMaxVelocity(RobotMap.climberMaxVel, RobotMap.climberSmartMotionPort);
        // climberPIDController.setSmartMotionMinOutputVelocity(RobotMap.climberMinVel, RobotMap.climberSmartMotionPort);
        // climberPIDController.setSmartMotionMaxAccel(RobotMap.climberMaxAcc, RobotMap.climberSmartMotionPort);
        // climberPIDController.setSmartMotionAllowedClosedLoopError(RobotMap.climberAllowedErr,
        //         RobotMap.climberSmartMotionPort);
            
        // limitSwitch1 = new DigitalInput(RobotMap.limitSwitchPortA);
        // limitSwitch2 = new DigitalInput(RobotMap.limitSwitchPortB);
        // limitSwitch3 = new DigitalInput(RobotMap.limitSwitchPortC);
        // limitSwitch4 = new DigitalInput(RobotMap.limitSwitchPortD);
                
        // motorBrake = new DoubleSolenoid(RobotMap.climberPistonPortA, RobotMap.climberPistonPortB);

        // shooterPIDController.setP(RobotMap.shooterPValue);
        // shooterPIDController.setI(RobotMap.shooterIValue);
        // shooterPIDController.setD(RobotMap.shooterDValue);
        // shooterPIDController.setIZone(RobotMap.shooterIZValue);
        // shooterPIDController.setFF(RobotMap.shooterFFValue);
        // shooterPIDController.setOutputRange(RobotMap.minOutput, RobotMap.maxOutput);
    }

    @Override
    public void initDefaultCommand() {

    }

    // public void createTelemetry() {
    //     SmartDashboard.putNumber("Climber: P Value", RobotMap.climberPValue);
    //     SmartDashboard.putNumber("Climber: I Value", RobotMap.climberIValue);
    //     SmartDashboard.putNumber("Climber: D Value", RobotMap.climberDValue);
    //     SmartDashboard.putNumber("Climber: FF Values", RobotMap.climberFFValue);
    //     SmartDashboard.putNumber("Climber: IZ Value", RobotMap.climberIZValue);
    // }

    // public void updateTelemetry() {
    //     if (climberPIDController.getP() != SmartDashboard.getNumber("Climber: P Value", RobotMap.climberPValue)) {
    //         climberPIDController.setP(SmartDashboard.getNumber("Climber: P Value", RobotMap.climberPValue));
    //     } else if (climberPIDController.getI() != SmartDashboard.getNumber("Climber: I Value",
    //             RobotMap.climberIValue)) {
    //         climberPIDController.setI(SmartDashboard.getNumber("Climber: I Value", RobotMap.climberIValue));
    //     } else if (climberPIDController.getD() != SmartDashboard.getNumber("Climber: D Value",
    //             RobotMap.climberDValue)) {
    //         climberPIDController.setD(SmartDashboard.getNumber("Climber: D Value", RobotMap.climberDValue));
    //     } else if (climberPIDController.getFF() != SmartDashboard.getNumber("Climber: FF Value",
    //             RobotMap.climberFFValue)) {
    //         climberPIDController.setFF(SmartDashboard.getNumber("Climber: FF Value", RobotMap.climberFFValue));
    //     } else if (climberPIDController.getIZone() != SmartDashboard.getNumber("Climber: IZ Value",
    //             RobotMap.climberIZValue)) {
    //         climberPIDController.setIZone(SmartDashboard.getNumber("Climber: IZ Value", RobotMap.climberIZValue));
    //     }
    // }

    // public DoubleSolenoid.Value getBrake() {
    //     return motorBrake.get();
    // }

    // public void setBrake(DoubleSolenoid.Value state) {
    //     motorBrake.set(state);
    // }

    // public DoubleSolenoid.Value getBrakeOppositeState(DoubleSolenoid.Value solenoid) {
    //     if (solenoid.equals(pistonIn)) {
    //        return pistonOut;
    //     } else {
    //        return pistonIn;
    //     }
    // }

    // public void toggleBrake() {
    //     setBrake(getBrakeOppositeState(getBrake()));
    // }
  
    // public void runTMP(double setpoint) {
    //     climberPIDController.setReference(setpoint, ControlType.kSmartMotion);
    // }

    public void setShooterReference(double setPoint) {
        shooterPIDController.setReference(setPoint, com.revrobotics.CANSparkMax.ControlType.kVelocity);
        SmartDashboard.putNumber("Shooter: Process Variable", encoder.getVelocity());
        SmartDashboard.putNumber("Shooter: Setpoint of Current Shot", setPoint);
      }

    public void setPowerOutput(double power) {
        shooterMotor.set(power);
    }

    

   
    
    

    
}
