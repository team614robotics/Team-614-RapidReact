// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.chassis.*;
import frc.robot.subsystems.chassis.Drivetrain;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.pneumatics.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.feeder.*;
import frc.robot.commands.feeder.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.*;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.List;
import frc.robot.subsystems.chassis.*;
import com.revrobotics.ColorSensorV3;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

public class Robot extends TimedRobot {
  public static XboxController m_controller = new XboxController(0);
  public static Drivetrain m_drivetrain = new Drivetrain();
  public static Climber m_climber = new Climber();
  public static Intake m_intake;
  public static Shooter m_shooter;
  public static Feeder m_feeder;
  public static int ballNumber;
  public static boolean highAuto;
  public static int allianceColor;
  public static DriverStation ds;
  public static AddressableLED m_led;
  public static AddressableLEDBuffer m_ledBuffer;
  public static Vision m_limelight;
  //public static final DriverStation.Alliance R_ALLIANCE;
  Alliance bAlliance;
  Alliance rAlliance;
  Command m_oneBallAuto;
  Command m_twoBallAuto;
  Command m_oneBallAutoHigh;
  Command m_twoBallAutoHigh;

  public static AHRS m_navX;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private SlewRateLimiter m_speedLimiter;
  private SlewRateLimiter m_rotLimiter;

  // An example trajectory to follow during the autonomous period.
  private Trajectory m_trajectory;

  // The Ramsete Controller to follow the trajectory.
  private RamseteController m_ramseteController;

  // The timer to use during the autonomous period.
  private Timer m_timer;

  // Create Field2d for robot and trajectory visualizations.
  private Field2d m_field;

  private static I2C.Port i2cPort = I2C.Port.kMXP;
  

  
  public static Pneumatics pneumatics;
  public static Ramsete m_ramsete;
  

  //public static ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  Command m_autonomousCommand;
  //Command m_getColor;
  SendableChooser autoChooser;
  SendableChooser autoHeightChooser;

  public static OI m_oi;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  // public final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  // public final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotInit() {
    
    try {
      m_navX = new AHRS(SPI.Port.kMXP, (byte) 200);
    } catch (RuntimeException e) {
      DriverStation.reportError("NAVX ERROR: " + e.getMessage(), true);
    }
    m_speedLimiter = new SlewRateLimiter(3);
    m_rotLimiter = new SlewRateLimiter(3);
    m_ramseteController = new RamseteController();
    

    pneumatics = new Pneumatics();

    m_limelight = new Vision();
    

    m_intake = new Intake();
    m_shooter = new Shooter();
    m_feeder = new Feeder();
    m_oi = new OI();
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(61);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 255, 0);
    }
    m_led.start();
    

    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
            new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // Push the trajectory to Field2d.
    m_field.getObject("traj").setTrajectory(m_trajectory);
    m_ramsete = new Ramsete();
    m_oneBallAuto = new OneBallAuto();
    m_twoBallAuto = new TwoBallAutoLow();
    m_oneBallAutoHigh = new OneBallAutoHigh();
    m_twoBallAutoHigh = new TwoBallAutoHigh();
    autoChooser = new SendableChooser<>();
    autoHeightChooser = new SendableChooser<>();
    ballNumber = 1;
    highAuto = false;
    autoChooser.setDefaultOption("1 Ball Low", m_oneBallAuto);
    autoChooser.addOption("2 Ball Low", m_twoBallAuto);
    autoChooser.setDefaultOption("1 Ball High", m_oneBallAutoHigh);
    autoChooser.addOption("2 Ball High", m_twoBallAutoHigh);
    autoHeightChooser.setDefaultOption("Low", false);
    autoHeightChooser.addOption("High", true);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    //SmartDashboard.putData("Auto Height Chooser", autoHeightChooser);
    Robot.m_limelight.setPipeline(1);
		Robot.m_limelight.setCamMode(1);
    Robot.m_limelight.setLED(1);
  }

  @Override
  public void robotPeriodic(){
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  @Override
  public void autonomousInit(){
    SmartDashboard.putData("Auto Chooser", autoChooser);
    Robot.m_limelight.setPipeline(1);
		Robot.m_limelight.setCamMode(1);
    Robot.m_limelight.setLED(1);
    //SmartDashboard.putNumber("Disabled", 0);
    // SmartDashboard.putNumber("Left Encoder Values", Robot.m_drivetrain.leftMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.leftMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.leftMotorB.getEncoder().setPosition(0);
    // SmartDashboard.putNumber("Right Encoder Values", Robot.m_drivetrain.rightMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.rightMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.rightMotorB.getEncoder().setPosition(0);
    Robot.m_intake.intakeMotor.setSmartCurrentLimit(RobotMap.intakeCurrentLimit);
    if (ds.getAlliance().toString().toLowerCase().equals("blue")) {
      SmartDashboard.putString("Alliance Color: ", "Blue");
      allianceColor = 1;
    }
    else if (ds.getAlliance().toString().toLowerCase().equals("red")) {
      SmartDashboard.putString("Alliance Color: ", "Red");
      allianceColor = 2;
    }
    // Initialize the timer.
    m_timer = new Timer();
    m_timer.start();
    //m_oi.getAutonomousCommand().schedule();
    // Reset the drivetrain's odometry to the starting pose of the trajectory.
    // SmartDashboard.putNumber("Distance Covered (Right Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.rightMotorA.getEncoder().getPosition()));
    // SmartDashboard.putNumber("Distance Covered (Left Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition()));
    
    // SmartDashboard.putNumber("Output (Left Wheels)", 0);
    // SmartDashboard.putNumber("Output (Right Wheels)", 0);
    // SmartDashboard.putNumber("Heading ", 0);
    // SmartDashboard.putNumber("Angle ", Robot.m_navX.getAngle());
    
    // if (ballNumber == 1){
    //   m_autonomousCommand = new OneBallAuto();
    // } else if (ballNumber == 2){
    //   m_autonomousCommand = new AutoMove();
    // }
    m_autonomousCommand = (Command) autoChooser.getSelected();
    //m_shooter.highShot = (boolean) autoHeightChooser.getSelected();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }
  
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Update odometry.

    // Update robot position on Field2d.
    
    //SmartDashboard.putNumber("Right Encoder Values", Robot.m_drivetrain.rightMotorA.getEncoder().getPosition());
    //SmartDashboard.putNumber("Left Encoder Values", Robot.m_drivetrain.leftMotorA.getEncoder().getPosition());
    Scheduler.getInstance().run();

    
    // m_drivetrain.updateOdometry();
  }

  @Override

  public void teleopInit() {
    SmartDashboard.putData("Auto Chooser", autoChooser);
    Robot.m_limelight.setPipeline(1);
		Robot.m_limelight.setCamMode(1);
    Robot.m_limelight.setLED(1);
    
    Robot.m_drivetrain.leftMotorA.setIdleMode(CANSparkMax.IdleMode.kCoast);
    Robot.m_drivetrain.leftMotorB.setIdleMode(CANSparkMax.IdleMode.kCoast);
    Robot.m_drivetrain.rightMotorA.setIdleMode(CANSparkMax.IdleMode.kCoast);
    Robot.m_drivetrain.rightMotorB.setIdleMode(CANSparkMax.IdleMode.kCoast);
    //SmartDashboard.putNumber("Disabled", 0);
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for green
      m_ledBuffer.setRGB(i, 0, 255, 0);
    }
    //SmartDashboard.putNumber("Left Encoder Values", Robot.m_drivetrain.leftMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.leftMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.leftMotorB.getEncoder().setPosition(0);
    //SmartDashboard.putNumber("Right Encoder Values", Robot.m_drivetrain.rightMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.rightMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.rightMotorB.getEncoder().setPosition(0);
    
    if (ds.getAlliance().toString().toLowerCase().equals("blue")) {
      SmartDashboard.putString("Alliance Color: ", "Blue");
      allianceColor = 1;
    }
    else if (ds.getAlliance().toString().toLowerCase().equals("red")) {
      SmartDashboard.putString("Alliance Color: ", "Red");
      allianceColor = 2;
    }
    
    // SmartDashboard.putNumber("Distance Covered (Right Wheels) (In Feet)",
    //     Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.rightMotorA.getEncoder().getPosition()));
    // SmartDashboard.putNumber("Distance Covered (Left Wheels) (In Feet)",
    //     Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition()));
    // m_getColor = new GetColor();
    Feeder.setBall1Type(0);
    Feeder.setBall2Type(0);
    //m_getColor.start();
    for (var i = 0; i < Robot.m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      Robot.m_ledBuffer.setRGB(i, 0, 255, 0);
    }
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
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    // final var xSpeed = -m_speedLimiter.calculate(m_controller.getLeftY()) *
    // Drivetrain.kMaxSpeed;

    Scheduler.getInstance().run();
    
    //SmartDashboard.putNumber("Right Encoder Values", Robot.m_drivetrain.rightMotorA.getEncoder().getPosition());
    //SmartDashboard.putNumber("Left Encoder Values", Robot.m_drivetrain.leftMotorA.getEncoder().getPosition());

    /*SmartDashboard.putNumber("Distance Covered (Right Wheels) (In Feet)",
    Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.rightMotorA.getEncoder().getPosition()));
SmartDashboard.putNumber("Distance Covered (Left Wheels) (In Feet)",
    Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition()));
    */
    //SmartDashboard.putData("Auto Number Chooser", autoChooser);
    //SmartDashboard.putData("Auto Height Chooser", autoHeightChooser);
//SmartDashboard.putNumber("Ball 1 Type", Feeder.checkBall1());
    // if (Feeder.checkBall1()==1){
    //   // Shuffleboard.getTab("Balls").addBoolean("e", RobotMap.colorTrue).withProperties("ColorWhenTrue", Color.kBlue);
    // }
    
//SmartDashboard.putNumber("Ball 2 Type", Feeder.checkBall2());
    //m_led.setData(m_ledBuffer);
    //SmartDashboard.putNumber("ToF Range", m_feeder.timeOfFlightSensor.getRange());
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    // final var rot = -m_rotLimiter.calculate(m_controller.getRightX()) *
    // Drivetrain.kMaxAngularSpeed;

    // m_drivetrain.drive(xSpeed, rot);
  }
  @Override
  public void disabledInit(){
    SmartDashboard.putData("Auto Chooser", autoChooser);
    Robot.m_limelight.setPipeline(1);
		Robot.m_limelight.setCamMode(1);
    Robot.m_limelight.setLED(1);
    OI.operatorController.setRumble(RumbleType.kRightRumble, RobotMap.rumbleOff);
		OI.driverController.setRumble(RumbleType.kRightRumble, RobotMap.rumbleOff);
		OI.operatorController.setRumble(RumbleType.kLeftRumble, RobotMap.rumbleOff);
		OI.driverController.setRumble(RumbleType.kLeftRumble, RobotMap.rumbleOff);
    //SmartDashboard.putNumber("Disabled", 1);
  }
  @Override
  public void disabledPeriodic(){
    SmartDashboard.putData("Auto Chooser", autoChooser);
    if (ds.getAlliance().toString().toLowerCase().equals("blue")) {
      SmartDashboard.putString("Alliance Color: ", "Blue");
      allianceColor = 1;
    }
    else if (ds.getAlliance().toString().toLowerCase().equals("red")) {
      SmartDashboard.putString("Alliance Color: ", "Red");
      allianceColor = 2;
    }
  }
}
