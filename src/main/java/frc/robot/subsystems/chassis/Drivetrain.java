// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.chassis.ArcadeDrive;
//import frc.robot.commands.chassis.ModifiedCurvatureDrive;

import frc.robot.RobotMap;

import java.util.ArrayList;
import frc.robot.Robot;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.ControlType;
import frc.robot.commands.chassis.ModifiedArcadeDrive;






public class Drivetrain extends Subsystem {

    public CANSparkMax leftMotorA, leftMotorB, rightMotorA, rightMotorB;
    public static DifferentialDrive drivetrain;

    
    public static double P, I, D, V;
    public ArrayList<Double> velocity, leftDistance, rightDistance;



    public CANPIDController pidControllerLeft, pidControllerrRight;
    public CANEncoder encoderLeft, encoderRight;

    static final double turnTolerance = 0.1f;


    public Drivetrain() {
        leftMotorA = new CANSparkMax(RobotMap.leftMotorAPort, RobotMap.brushless);
        leftMotorB = new CANSparkMax(RobotMap.leftMotorBPort, RobotMap.brushless);
        rightMotorA = new CANSparkMax(RobotMap.rightMotorAPort, RobotMap.brushless);
        rightMotorB = new CANSparkMax(RobotMap.rightMotorBPort, RobotMap.brushless);

        drivetrain = new DifferentialDrive(leftMotorA, rightMotorA);
        
        leftMotorA.setInverted(true);
        rightMotorA.setInverted(false);
        leftMotorB.follow(leftMotorA);
        rightMotorB.follow(rightMotorA);
        
        // leftMotorA.getPIDController().setP(5e-5);
        // leftMotorA.getPIDController().setI(1e-6);
        // leftMotorA.getPIDController().setD(0);
        // leftMotorA.getPIDController().setIZone(0);
        // leftMotorA.getPIDController().setFF(0);
        // leftMotorA.getPIDController().setOutputRange(-1, 1);
        
        // rightMotorA.getPIDController().setP(5e-6);
        // rightMotorA.getPIDController().setI(1e-6);
        // rightMotorA.getPIDController().setD(0);
        // rightMotorA.getPIDController().setIZone(0);
        // rightMotorA.getPIDController().setFF(0);
        // rightMotorA.getPIDController().setOutputRange(-1, 1);

        // leftMotorA.getPIDController().setSmartMotionMaxVelocity(2000, 0);
        // leftMotorA.getPIDController().setSmartMotionMinOutputVelocity(0, 0);
        // leftMotorA.getPIDController().setSmartMotionMaxAccel(1000, 0);
        // leftMotorA.getPIDController().setSmartMotionAllowedClosedLoopError(0, 0);

        // rightMotorA.getPIDController().setSmartMotionMaxVelocity(3000, 0);
        // rightMotorA.getPIDController().setSmartMotionMinOutputVelocity(0, 0);
        // rightMotorA.getPIDController().setSmartMotionMaxAccel(1000, 0);
        // rightMotorA.getPIDController().setSmartMotionAllowedClosedLoopError(0, 0);

        

        
    }

    public void initDefaultCommand() {
        setDefaultCommand(new ModifiedArcadeDrive());
    }

    public void resetPath() {
        // PathGenerator.newPoints.clear();
        // PathGenerator.newVectors.clear();
        // PathGenerator.finalPoints.clear();
        // PathGenerator.newNumOPoints.clear();
        // SmoothPosition.newPathPoints.clear();
        // SmoothPosition.pathPoints.clear();
        // KinematicsCalculator.curvature.clear();
        // KinematicsCalculator.distance.clear();
        // KinematicsCalculator.leftDistance.clear();
        // KinematicsCalculator.leftVelocity.clear();
        // KinematicsCalculator.outputs.clear();
        // KinematicsCalculator.rightDistance.clear();
        // KinematicsCalculator.rightVelocity.clear();
        // KinematicsCalculator.velocity.clear();
        // SmoothVelocity.leftVelocities.clear();
        // SmoothVelocity.rightVelocities.clear();
        // TimeStepCalculator.timeOutlined.clear();
        // velocity.clear();
        // leftDistance.clear();
        // rightDistance.clear();
    }

    

    public void arcadeDrive(double speed, double rotateValue) {
        drivetrain.arcadeDrive(speed, rotateValue);
    }


    public double distanceInFeet(double encoderValue) {
        return encoderValue * (((RobotMap.wheelDiameter / 12) * Math.PI) / RobotMap.ticksInARevolution);
    }

    public void resetDrivetrain() {
        Robot.m_navX.reset();
        leftMotorA.set(0);
        leftMotorB.set(0);
        rightMotorA.set(0);
        rightMotorB.set(0);
    }

    public double getAngle() {
        return Robot.m_navX.getAngle();
    }


    
}