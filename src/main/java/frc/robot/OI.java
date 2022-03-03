/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//poopy butt face
package frc.robot;
import java.time.Duration;

import com.fasterxml.jackson.databind.deser.impl.SetterlessProperty;
import frc.robot.commands.climber.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.intake.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.commands.shooter.*;
import frc.robot.commands.feeder.*;
import frc.robot.commands.intake.*;

//import frc.robot.commands.intake.runOuttake;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
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



  //Driver Controller 
  public static final XboxController driverController = new XboxController(0);

  public static final Button RunIntake = new JoystickButton(driverController, RobotMap.LeftBumper);
  public static final Button ReverseIntake = new JoystickButton(driverController, RobotMap.XButton);
  public static final Button IntakeNoColor = new JoystickButton(driverController, RobotMap.BButton);
  

  // public static final Button RunFeeder = new JoystickButton(driverController, RobotMap.AButton);
  public static final Button ToggleIntake = new JoystickButton(driverController, RobotMap.RightBumper);
  // public static final Button ReverseFeeder = new JoystickButton(driverController, RobotMap.BackButton);
  public static final Button ColorSensor = new JoystickButton(driverController, RobotMap.BackButton);

  //Operator controller
  public static final XboxController operatorController = new XboxController(1);
  
  public static final Button MoveClimberUpwards = new JoystickButton(operatorController, RobotMap.YButton);
  public static final Button MoveClimberDownwards = new JoystickButton(operatorController, RobotMap.XButton);
  public static final Button AccelerateFlywheel = new JoystickButton(operatorController, RobotMap.BButton);
  public static final Button ShootLowWithLogic = new JoystickButton(operatorController, RobotMap.LeftBumper);
  public static final Button ShootHighWithLogic = new JoystickButton(operatorController, RobotMap.RightBumper);
  // public static final Button AccelerateFlywheel = new JoystickButton(operatorController, RobotMap.BButton);
  public static final Button RunFeeder = new JoystickButton(operatorController, RobotMap.AButton);
  public static final Button ReverseFeeder = new JoystickButton(operatorController, RobotMap.BackButton);
  public static final Button ReverseShooter = new JoystickButton(operatorController, RobotMap.StartButton);

  public OI() {
    /* Driver Controller */
    RunIntake.whileHeld(new RunIntakeBasic(RobotMap.reverseIntakeSpeed, RobotMap.doIntakeColor));
    IntakeNoColor.whileHeld(new RunIntakeBasic (RobotMap.reverseIntakeSpeed, RobotMap.doNotIntakeColor));
    ToggleIntake.whenPressed(new IntakeToggle());
    ReverseIntake.whileHeld(new RunIntakeBasic(RobotMap.intakeSpeed, RobotMap.doNotIntakeColor));
    ColorSensor.whileHeld(new GetColor());

    /* Operator Controller */
    MoveClimberUpwards.whileHeld(new MoveClimberUpwards());
    MoveClimberDownwards.whileHeld(new MoveClimberDownwards());
    ShootLowWithLogic.whileHeld(new RunShooterLowLogic());
    ShootHighWithLogic.whileHeld(new RunShooterHighWithLogic());
    AccelerateFlywheel.whileHeld(new AccelerateFlywheel());
    RunFeeder.whileHeld(new RunFeeder(RobotMap.feederSpeed));
    ReverseFeeder.whileHeld(new RunFeeder(RobotMap.reverseFeederSpeed));
    ReverseShooter.whileHeld(new ReverseShooter(RobotMap.reverseShooterSpeed));
  }
}
