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
//import frc.robot.commands.shooter.LowShotTrigger;

import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import com.playingwithfusion.*;

public class Shooter extends Subsystem {
    public CANSparkMax shooterMotor;
    SparkMaxPIDController shooterPIDController;
    private RelativeEncoder encoder;
    public static double last_accel_x; 
    public static double last_accel_y;
    public boolean shooting;
    public boolean highShot;

    public Shooter() {
        shooterMotor = new CANSparkMax(RobotMap.shooterMotorPort, MotorType.kBrushless);
        shooterMotor.restoreFactoryDefaults();
        shooterPIDController = shooterMotor.getPIDController();

        shooterPIDController.setP(RobotMap.shooterPValueLow,0);
        shooterPIDController.setI(RobotMap.shooterIValueLow,0);
        shooterPIDController.setD(RobotMap.shooterDValueLow,0);
        //shooterPIDController.setIZone(RobotMap.shooterIZValue);
        shooterPIDController.setFF(RobotMap.shooterFFValueLow,0);
        
        shooterPIDController.setP(RobotMap.shooterPValueHigh,1);
        shooterPIDController.setI(RobotMap.shooterIValueHigh,1);
        shooterPIDController.setD(RobotMap.shooterDValueHigh,1);
        //shooterPIDController.setIZone(RobotMap.shooterIZValue);
        shooterPIDController.setFF(RobotMap.shooterFFValueHigh,1);

        shooterPIDController.setOutputRange(RobotMap.minOutput, RobotMap.maxOutput);
        encoder = shooterMotor.getEncoder();
        shooting = false;
        highShot = false;
    }

    @Override
    public void initDefaultCommand() {
            //setDefaultCommand(new LowShotTrigger(RobotMap.shooterVelocitySetpointOurs));
    }

    public static void setAccel_x(Double acx) {
        last_accel_x = acx;
    }
    public static double getAccel_x() {
        return last_accel_x;
    }
    public static void setAccel_y(Double acy) {
        last_accel_x = acy;
    }
    public static double getAccel_y() {
        return last_accel_y;
    }

    public void setHighShot() {
        //shooterPIDController.s
    }

    public void setLowShot() {

    }

    public void setShooterReference(double setPoint) {
        shooterPIDController.setReference(setPoint, com.revrobotics.CANSparkMax.ControlType.kVelocity);
        //SmartDashboard.putNumber("Shooter: Process Variable", encoder.getVelocity());
        //SmartDashboard.putNumber("Shooter: Setpoint of Current Shot", setPoint);
      }
    
    // public void setLowValues() {
    //     shooterPIDController.setP(RobotMap.shooterPValueLow);
    //     shooterPIDController.setI(RobotMap.shooterIValueLow);
    //     shooterPIDController.setD(RobotMap.shooterDValueLow);
    //     //shooterPIDController.setIZone(RobotMap.shooterIZValue);
    //     shooterPIDController.setFF(RobotMap.shooterFFValueLow);
    // }

    // public void setHighValues() {
    //     shooterPIDController.setP(RobotMap.shooterPValueHigh);
    //     shooterPIDController.setI(RobotMap.shooterIValueHigh);
    //     shooterPIDController.setD(RobotMap.shooterDValueHigh);
    //     //shooterPIDController.setIZone(RobotMap.shooterIZValue);
    //     shooterPIDController.setFF(RobotMap.shooterFFValueHigh);
    // }

    public void setPowerOutput(double power) {
        shooterMotor.set(power);
    }
  
}
