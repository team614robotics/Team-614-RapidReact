// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.chassis.AutoMove;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.chassis.AutoMove;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.pneumatics.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.feeder.*;
import frc.robot.commands.feeder.*;



import com.revrobotics.ColorSensorV3;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

public class Robot extends TimedRobot {
  public static XboxController m_controller = new XboxController(0);
  public static Drivetrain m_drivetrain = new Drivetrain();
  public static Climber m_climber = new Climber();
  public static Intake m_intake;
  public static Shooter m_shooter;
  public static Feeder m_feeder;
  public static int allianceColor;
  public static DriverStation ds;
  //public static final DriverStation.Alliance R_ALLIANCE;
  Alliance bAlliance;
  Alliance rAlliance;

  private static I2C.Port i2cPort = I2C.Port.kMXP;


  public static AHRS m_navX;
  public static Pneumatics pneumatics;

  public static ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  Command m_autonomousCommand;
  Command m_getColor;
  // SendableChooser allianceChooser;

  public static OI m_oi;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  public final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  public final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotInit() {

    try {
      m_navX = new AHRS(SPI.Port.kMXP, (byte) 200);
    } catch (RuntimeException e) {
      DriverStation.reportError("NAVX ERROR: " + e.getMessage(), true);
    }
    pneumatics = new Pneumatics();

    // allianceChooser = new SendableChooser<>();
    // allianceColor = 0;
    // allianceChooser.addOption("Blue Alliance", allianceColor = 1);
    // allianceChooser.addOption("Red Alliance", allianceColor = 2);

    m_intake = new Intake();
    m_shooter = new Shooter();
    m_feeder = new Feeder();
    m_oi = new OI();
    
  }

  @Override
  public void autonomousInit(){
    
    // SmartDashboard.putNumber("Left Encoder Values", Robot.m_drivetrain.leftMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.leftMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.leftMotorB.getEncoder().setPosition(0);
    // SmartDashboard.putNumber("Right Encoder Values", Robot.m_drivetrain.rightMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.rightMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.rightMotorB.getEncoder().setPosition(0);
    if (ds.getAlliance().toString().toLowerCase().equals("blue")) {
      SmartDashboard.putString("Alliance: ", "Blue");
      allianceColor = 1;
    }
    else if (ds.getAlliance().toString().toLowerCase().equals("red")) {
      SmartDashboard.putString("Alliance: ", "Red");
      allianceColor = 2;
    }
    // SmartDashboard.putNumber("Distance Covered (Right Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.rightMotorA.getEncoder().getPosition()));
    // SmartDashboard.putNumber("Distance Covered (Left Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition()));
    
    // SmartDashboard.putNumber("Output (Left Wheels)", 0);
    // SmartDashboard.putNumber("Output (Right Wheels)", 0);
    // SmartDashboard.putNumber("Heading ", 0);
    // SmartDashboard.putNumber("Angle ", Robot.m_navX.getAngle());
    
    m_autonomousCommand = new AutoMove();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }
  
  @Override
  public void autonomousPeriodic() {
    
    
    Scheduler.getInstance().run();

    
    // m_drivetrain.updateOdometry();
  }

  @Override

  public void teleopInit() {
    Robot.m_drivetrain.leftMotorA.setIdleMode(CANSparkMax.IdleMode.kCoast);
    Robot.m_drivetrain.leftMotorB.setIdleMode(CANSparkMax.IdleMode.kCoast);
    Robot.m_drivetrain.rightMotorA.setIdleMode(CANSparkMax.IdleMode.kCoast);
    Robot.m_drivetrain.rightMotorB.setIdleMode(CANSparkMax.IdleMode.kCoast);

    // SmartDashboard.putNumber("Left Encoder Values", Robot.m_drivetrain.leftMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.leftMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.leftMotorB.getEncoder().setPosition(0);
    // SmartDashboard.putNumber("Right Encoder Values", Robot.m_drivetrain.rightMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.rightMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.rightMotorB.getEncoder().setPosition(0);
    
    if (ds.getAlliance().toString().toLowerCase().equals("blue")) {
      SmartDashboard.putString("Alliance: ", "Blue");
      allianceColor = 1;
    }
    else if (ds.getAlliance().toString().toLowerCase().equals("red")) {
      SmartDashboard.putString("Alliance: ", "Red");
      allianceColor = 2;
    }

    // SmartDashboard.putNumber("Distance Covered (Right Wheels) (In Feet)",
    //     Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.rightMotorA.getEncoder().getPosition()));
    // SmartDashboard.putNumber("Distance Covered (Left Wheels) (In Feet)",
    //     Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition()));
    m_getColor = new GetColor();
    Feeder.setBall1Type(0);
    Feeder.setBall2Type(0);
    m_getColor.start();
    // SmartDashboard.putNumber("Output (Left Wheels)", 0);
    // SmartDashboard.putNumber("Output (Right Wheels)", 0);
    // SmartDashboard.putNumber("Heading ", 0);
    // SmartDashboard.putNumber("Angle ", Robot.m_navX.getAngle());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    // final var xSpeed = -m_speedLimiter.calculate(m_controller.getLeftY()) *
    // Drivetrain.kMaxSpeed;

    Scheduler.getInstance().run();
    
    // SmartDashboard.putData("Alliance Color", allianceChooser);
    SmartDashboard.putNumber("Ball 1 Type", Feeder.checkBall1());
    SmartDashboard.putNumber("Ball 2 Type", Feeder.checkBall2());
    
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    // final var rot = -m_rotLimiter.calculate(m_controller.getRightX()) *
    // Drivetrain.kMaxAngularSpeed;

    // m_drivetrain.drive(xSpeed, rot);
  }
}
