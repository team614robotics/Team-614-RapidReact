package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.chassis.ModifiedArcadeDrive;
import frc.robot.commands.feeder.RunFeeder;
import frc.robot.commands.shooter.AccelerateFlywheel;
import frc.robot.commands.shooter.*;
import frc.robot.commands.intake.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class TwoBallAutoLowWall extends CommandGroup {
    public TwoBallAutoLowWall() {
        addSequential(new IntakeToggle());
        addParallel(new AccelerateFlywheel(RobotMap.shooterVelocitySetpointOurs));
        addParallel(new AutoRunIntake(RobotMap.doNotIntakeColor, RobotMap.autoDriveTime));
        addSequential(new AutoArcadeEnocderDrive(38, RobotMap.forward, RobotMap.driveFast, 3));// was 42
        addSequential(new ResetEncoders());
        addSequential(new IntakeToggle());
        //addSequential(new Wait(3));
        addParallel(new AccelerateFlywheel(RobotMap.shooterVelocitySetpointOurs));
        addSequential(new AutoArcadeEnocderDrive(118, RobotMap.backward, RobotMap.driveFast, 4));
        //addSequential(new ResetEncoders());
        addSequential(new AutoShootLow(4));
        addSequential(new AutoArcadeEnocderDrive(110, RobotMap.forward, RobotMap.driveFast, 3));
        addSequential(new AutoArcadeRotate(Robot.m_navX, 125, RobotMap.counterClockwise, 2));
    }
}
