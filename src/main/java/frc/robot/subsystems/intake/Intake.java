/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.*;
import frc.robot.OI;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import frc.robot.subsystems.serializer.Serializer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.commands.intake.IntakeOnJoystick;
//import frc.robot.subsystems.compressorcontrol.*;
import frc.robot.commands.intake.RunIntakeBasic;
import frc.robot.commands.intake.oneButtonIntake;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;


public class Intake extends Subsystem {
   // Put methods for controlling this subsystem
   // here. Call these from Commands.

   // One Intake Motor, simple speed
   public CANSparkMax intakeMotor;
   private SparkMaxPIDController intakePIDController;
   private RelativeEncoder encoder;

   public DoubleSolenoid intakeSolenoidA;
   public DoubleSolenoid intakeSolenoidB;

   public DoubleSolenoid.Value pistonIn = DoubleSolenoid.Value.kReverse;
   public DoubleSolenoid.Value pistonOut = DoubleSolenoid.Value.kForward;

   public Intake() {
      intakeMotor = new CANSparkMax(RobotMap.intakeMotorPort, MotorType.kBrushless);
      intakeSolenoidA = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.doubleSolenoidAPort1, RobotMap.doubleSolenoidAPort2);
      intakeSolenoidB = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.doubleSolenoidBPort1, RobotMap.doubleSolenoidBPort2);
      intakeMotor.setSmartCurrentLimit(RobotMap.intakeCurrentLimit);
      //SmartDashboard.putNumber("Intake: Intake Speed", 0.0);

      intakePIDController = intakeMotor.getPIDController();

      intakePIDController.setP(RobotMap.intakePValue);
      intakePIDController.setI(RobotMap.intakeIValue);
      intakePIDController.setD(RobotMap.intakeDValue);
      //shooterPIDController.setIZone(RobotMap.shooterIZValue);
      intakePIDController.setFF(RobotMap.intakeFFValue);
      intakePIDController.setOutputRange(RobotMap.minOutput, RobotMap.maxOutput);
      encoder = intakeMotor.getEncoder();

   }

   @Override
   public void initDefaultCommand() {
      // setDefaultCommand(new oneButtonIntake(RobotMap.intakeSpeed));
   }

   public DoubleSolenoid.Value getDoubleSolenoidA() {
      return intakeSolenoidA.get();
   }

   public DoubleSolenoid.Value getDoubleSolenoidB() {
      return intakeSolenoidB.get();
   }


   public void setDoubleSolenoidA(DoubleSolenoid.Value state) {
      intakeSolenoidA.set(state);
   }

   public void setDoubleSolenoidB(DoubleSolenoid.Value state) {
      intakeSolenoidB.set(state);
   }

   public DoubleSolenoid.Value getOppositeState(DoubleSolenoid.Value solenoid) {
      if (solenoid.equals(pistonIn)) {
         return pistonOut;
      } else {
         return pistonIn;
      }
   }

   public void toggleDoubleSolenoidA() {
      setDoubleSolenoidA(getOppositeState(getDoubleSolenoidA()));
   }

   public void toggleDoubleSolenoidB() {
      setDoubleSolenoidB(getOppositeState(getDoubleSolenoidB()));
   }

   public void toggleIntake() {
      toggleDoubleSolenoidA();
      toggleDoubleSolenoidB();
      RobotMap.isDumb = !RobotMap.isDumb;
   }


   public void intakemotorPortSpeed(double speed) {
      intakeMotor.set(speed);
   }
   public void setIntakeReference(double setPoint) {
      intakePIDController.setReference(setPoint, com.revrobotics.CANSparkMax.ControlType.kVelocity);
      //SmartDashboard.putNumber("Shooter: Process Variable", encoder.getVelocity());
      //SmartDashboard.putNumber("Shooter: Setpoint of Current Shot", setPoint);
    }

   public double getCurrent() {
      return intakeMotor.getOutputCurrent();
   }

   public boolean checkCurrent(double current) {
      return intakeMotor.getOutputCurrent() > current ? true : false;
   }
   public void rumble(double value) {
		OI.operatorController.setRumble(GenericHID.RumbleType.kRightRumble, value);
		//OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, value);
		OI.operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, value);
		//OI.driverController.setRumble(GenericHID.RumbleType.kLeftRumble, value);
		}

      public void rumbleOff() {
         OI.operatorController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.rumbleOff);
         //OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.rumbleOff);
         OI.operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, RobotMap.rumbleOff);
         //OI.driverController.setRumble(GenericHID.RumbleType.kLeftRumble, RobotMap.rumbleOff);
         }

   // public void runSerializer(Serializer serializer, double current, double speed) {
   //    if (checkCurrent(current)) {
   //       serializer.runMotorFunction(speed);
   //    }
   // }
}