package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.chassis.ModifiedArcadeDrive;
import frc.robot.commands.shooter.*;
import frc.robot.commands.intake.*;

public class FiveBallAuto extends CommandGroup {
  public FiveBallAuto() {

    addSequential(new IntakeToggle());
    addParallel(new AccelerateFlywheel(RobotMap.shooterVelocitySetpointHigh));
    addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed, RobotMap.doNotIntakeColor, RobotMap.autoDriveTime));
    addSequential(new AutoArcadeEnocderDrive(38, RobotMap.forward, -.8));// was 42
    addSequential(new ResetEncoders());
    addSequential(new IntakeToggle());
    addSequential(new AutoArcadeEnocderDrive(38, RobotMap.backward, -8));
    addSequential(new ResetEncoders());
    addSequential(new AutoShootHigh(RobotMap.autoShootTime));
    addSequential(new AutoArcadeRotate(Robot.m_navX, 93, RobotMap.clockwise, 1));
    addSequential(new IntakeToggle());
    addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed, RobotMap.doNotIntakeColor, 3));
    addSequential(new AutoArcadeEnocderDrive(94, RobotMap.forward, RobotMap.driveFast));// was 42
    addSequential(new ResetEncoders());
    addSequential(new IntakeToggle());
    addSequential(new AutoArcadeRotate(Robot.m_navX, 31, RobotMap.counterClockwise, .5));
    addSequential(new AutoArcadeEnocderDrive(60, RobotMap.backward, RobotMap.driveFast));
    addSequential(new AutoShootHigh(RobotMap.autoShootTime));
    addSequential(new AutoArcadeRotate(Robot.m_navX, 31, RobotMap.counterClockwise, .5));
    addSequential(new IntakeToggle());
    addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed, RobotMap.doNotIntakeColor, 3));
    addSequential(new AutoArcadeEnocderDrive(185, RobotMap.forward, -.9));
    addSequential(new ResetEncoders());
    addSequential(new IntakeToggle());
    addSequential(new AutoArcadeEnocderDrive(185, RobotMap.backward, -.9));

  }
}