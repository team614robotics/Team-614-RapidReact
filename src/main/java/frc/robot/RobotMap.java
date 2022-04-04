/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;

  public static final String RobotName = "Fax Machine";

  // Alliance
  public static final int BlueAlliance = 1;
  public static final int RedAlliance = 2;

  // Controller
  // public static final XboxController intakeController;

  public static boolean isDumb = true;// intake piston activation
  public static boolean intakeStart = false;
  public static boolean oneBall = false;
  public static boolean twoBall = false;
  public static boolean ToF;

  public static final double autoDriveTime = 2;
  public static final double autoDriveTime2 = 4;
  public static final int autoShootTime = 1;


  //One Ball Low 
  public static final int autoOneBallLowShootTime = 4;
  public static final int autoOneBallHighShootTime = 4;
  public static final int autoTwoBallLowShootTime = 4;
  public static final int autoTwoBallHighShootTime = 4;
  public static final int autoThreeBallLowShootTime = 1;
  public static final int autoThreeBallHighShootTime = 1;

  // 68.57318878173828
  // 67.83502197265625

  // Chassis
  public static final double ticksPerInch = 0.6024925;// 68.57318878173828 ticks/ 120 inch
  public static final int leftMotorAPort = 1; // previously 1
  public static final int leftMotorBPort = 2; // previously 2
  public static final int rightMotorAPort = 3; // previously 3
  public static final int rightMotorBPort = 4; // next 12, previously 4
  public static final MotorType brushless = MotorType.kBrushless;
  public static final MotorType brushed = MotorType.kBrushed;
  public static final double ticksInARevolution = 10.659;
  public static final double wheelDiameter = 5.7;
  public static final double pValue = 0.3855;
  public static final double sumPValue = 0.03;
  public static final double iValue = 0.0;
  public static final double sumDValue = 0.000090;
  public static final double dValue = 0.0;
  public static final double fValue = 0.0001;
  public static double vValue = 0.08;
  public static final double hValue = -0.01;
  public static final double chassisVelocityPValue = 5e-5;
  public static final double chassisVelocityIValue = 1e-6;
  public static final double chassisVelocityDValue = 0;
  public static final double chassisVelocityFFValue = 0;
  public static final double chassisVelocityIZValue = 0;
  public static final double chassisVelocityMinOutput = -1;
  public static final double chassisVelocityMaxOutput = 1;
  public static final double autoArcadeSpeed = -0.5;// was -.5
  public static final double autoRotateValue = -0.11;
  public static final double kWheelDiameterMeters = 0.1524;
  public static final double encoderPerFeet = -6.766666666;
  public static final double rampD = .5;// was .5 deciedes the ramping of the driveVal ramp = 1 == x^3, ramp = 0 == x,
  public static final double rampR = .8;// was .5 deciedes the ramping of the rotateVal ramp = 1 == x^3, ramp = 0 == x,
  public static final double rotateFast = .4;
  public static final double rotateProp = .8;
  public static final double rotateSlow = .1;
  public static final double driveFast = -.5;
  public static final double driveProp = .9;// was .9
  public static final double driveSlow = -.4;// .3
  public static final double rotateTime1 = .9;// was .9
  public static final double rotateTime2 = -.4;// .3
  public static final double pidAngleThreshold = 1;

  public static final double ks = 0.2085;// 0.11314
  public static final double kv = 0.26409;// 0.25777
  public static final double ka = 0.049162;// 0.15984
  public static final double kpVelocity = 0.36134;// 0.3855

  public static final double staticArcadeDriveValue = 0.8;

  public static final boolean forward = true;
  public static final boolean backward = false;

  public static final boolean clockwise = true;
  public static final boolean counterClockwise = false;

  public static final double kTrackwidthMeters = 0.4572;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAngularSpeed = 2 * Math.PI;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;

  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  // Shooter
  public static final double shooterPValueLow = 0.0004;//0.00008;
  public static final double shooterIValueLow = 0;
  public static final double shooterDValueLow = 0.001;//0.001;
  // public static final double shooterIZValue = 0;
  public static final double shooterFFValueLow = 0.00018;//0.00023;

  public static final double shooterPValueHigh = 0.00025;//0.00008;
  public static final double shooterIValueHigh = 0;
  public static final double shooterDValueHigh = 0.001;//0.001;
  public static final double shooterFFValueHigh = 0.00018;//0.00023;

  public static final double maxOutput = 1;
  public static final double minOutput = -1;
  public static final double maxRPM = 5700;
  public static final int shooterMotorPort = 21;
  public static final double turnOffShooterMotor = 0;
  public static final double autoShooterSpeed = 0.5;
  public static final int shooterVelocitySetpointOurs = 2150; // was 1900 with old pid 
  public static final int driverShooterVelocitySetpointOurs = 2300;
  public static final int shooterVelocityThreshold = 2000;
  public static final int shooterVelocitySetpointHigh = 3800; //was 3600 with old pid// was 4000
  public static final int shooterVelocityThresholdHigh = 3600; // was 3800
  public static final int shooterVelocitySetpointOpposing = 2100;// changed from 1800
  public static final int shooterVelocityThresholdOpposing = 2200;// 2100
  public static final double collisionThreshold = 1;
  // public static final int acceleratorMotorPort = 7; //5 previous
  // public static double lowGoalSpeed = 4000;
  // public static double highGoalSpeed = 5000;

  public static final double reverseShooterSpeed = -0.5;

  // Climber
  public static final int limitSwitchPortA = 0;
  public static final int limitSwitchPortB = 1;

  // public static final int limitSwitchPortC = 2;
  // public static final int limitSwitchPortD = 3;
  public static final double MaxEncoderTick = 187;// 36:1 187
  public static final double MinEncoderTick = 2;
  public static final int climberMotorPort = 31; // actually 2
  public static final double climberPValue = 0.0008;
  public static final double climberIValue = 0;
  public static final double climberDValue = 0.001;
  // public static final double climberIZValue = 0;
  public static final double climberFFValue = 0.2;
  public static final int climberVelocitySetpoint = 8000;
  public static final int climberMaxVelocity = 8000;
  public static final double joystickClimbThreshold = 0.2;
  // public static final double climberMinOutput = -1;
  // public static final double climberMaxOutput = 1;
  // public static final double climberMaxVel = 2000;
  // public static final double climberMaxAcc = 200;
  // public static final double climberMinVel = 0;
  // public static final double climberAllowedErr = 0;
  // public static final int climberSmartMotionPort = 0;
  // public static final int climberPistonPortA = 0;
  // public static final int climberPistonPortB = 1;
  public static final double initalClimberMotor = 0;
  public static final double turnOffClimberMotor = 0;
  public static final double moveClimberDown = -0.9;
  public static final double moveClimberUp = 0.9;
  public static final boolean restricted = true;
  public static final boolean unRestricted = false;

  // Intake
  public static final double intakePValue = 0.000007;
  public static final double intakeIValue = 0;
  public static final double intakeDValue = 0.0001;
  public static final double intakeFFValue = 0.0000874;
  public static final int intakeMotorPort = 11; // 3 next? previously 8
  public static final int doubleSolenoidAPort1 = 4;
  public static final int doubleSolenoidAPort2 = 5;
  public static final int doubleSolenoidBPort1 = 6;// we will not remove double solenoid B
  public static final int doubleSolenoidBPort2 = 7;
  public static final double turnOffIntakeMotor = 0;
  public static final double intakeSpeed = -9120;// 1 works well with the 21:1
  public static final double autoIntakeSpeed = -9120;// 1 works well with the 21:1
  public static final double reverseIntakeSpeed = 9120;// was .6
  public static final int intakeCurrentLimit = 25;
  public static final boolean doIntakeColor = true;
  public static final boolean doNotIntakeColor = false;

  // //Vision
  // public static final double limelightToTarget = 25.7;
  // public static final double vPValue = 0.1;
  // public static final double vPAltValue = 0.023;
  // public static final double vFFValue = 0.35;
  // public static final double vMaxOutput = 0.5;
  // public static final double highGoalDistance = 74.4;
  // public static final DoubleSolenoid.Value PistonOut =
  // DoubleSolenoid.Value.kForward;
  // public static final DoubleSolenoid.Value PistonIn =
  // DoubleSolenoid.Value.kReverse;
  // public static final int useLEDModeInPipeline = 0;
  // public static final int forceOff = 1;
  // public static final int forceBlink = 2;
  // public static final int forceOn = 3;
  // public static final int visionProcessing = 0;
  // public static final int driverCamera = 1;
  // public static final int pipelineClose = 0;
  // public static final int pipelineFar = 1;
  // public static final int pipelineExtra = 2;

  // //pneumatics
  public static final int compressor = 0;

  // Controller.
  public static final int AButton = 1;
  public static final int BButton = 2;
  public static final int XButton = 3;
  public static final int YButton = 4;
  public static final int LeftBumper = 5;
  public static final int RightBumper = 6;
  public static final int BackButton = 7;
  public static final int StartButton = 8;
  public static final int LeftStick = 9;
  public static final int RightStick = 10;
  public static final int driverPort = 0;
  public static final int operatorPort = 2;
  public static final double triggerPressed = 0.5;

  // //Serializer
  // public static final int serializerMotorPortA = 3; //previously 9
  // public static final int serializerMotorPortB = 5; //10 previously
  // public static final int setCurrent = 30;

  // //Feeder
  public static final int feederMotorPort = 22; // actually 8
  public static final BooleanSupplier colorTrue = () -> true;
  // public static final double feederPValue = 5e-5;
  // public static final double feederIValue = 0;
  // public static final double feederDValue = 0;
  // public static final double feederIZValue = 0;
  // public static final double feederFFValue = 0;
  public static final double initalizeFeederMotor = 0;
  public static final double turnOffFeederMotor = 0;
  public static final int ballProximity = 100;
  public static final int colorMultiplier = 255;
  public static final double rumbleOff = 0;
  public static final double rumbleOurBall = 0;// 0.7;
  public static final double rumbleOpposingBall = 0;// 0.2;
  public static final double neutralRumble = 0.1;
  public static final double oneBallRumble = 0.2;
  public static final double twoBallRumble = 0.4;
  public static final int noBall = 0;
  public static final int blueBall = 1;
  public static final int redBall = 2;
  public static final int blueRValue = 0;
  public static final int blueGValue = 0;
  public static final int blueBValue = 255;
  public static final int redRValue = 255;
  public static final int redGValue = 0;
  public static final int redBValue = 0;
  public static final int defaultRValue = 0;
  public static final int defaultGValue = 255;
  public static final int defaultBValue = 0;
  public static final double feederSpeed = -0.2;
  public static final double feederShootSpeed = -0.8;
  public static final double feederTime = 0;
  public static final double reverseFeederSpeed = .7;
  public static final int colorTime = 5;
  public static final double continueFeederTime = 0.8;
  public static final int timeOfFlightPort = 0;
  public static final int timeOfFlightPort2 = 1;
  public static final int ToFRange = 300;

}