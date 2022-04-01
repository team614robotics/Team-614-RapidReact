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

public class TwoBallAutoLowWeird extends CommandGroup {
    int accelerationSpeed;
    public TwoBallAutoLowWeird() {
        //addSequential(new AutoShoot());
        //SmartDashboard.putNumber("2 Low", 1);
        // accelerationSpeed = RobotMap.shooterVelocitySetpointOurs;
        // addSequential(new IntakeToggle());
        // addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed, RobotMap.doNotIntakeColor,RobotMap.autoDriveTime));
        // addSequential(new AutoArcadeDrive(RobotMap.autoDriveTime, RobotMap.forward));
        // addSequential(new IntakeToggle());
        // addParallel(new AccelerateFlywheel(accelerationSpeed));
        // addSequential(new AutoArcadeDrive(RobotMap.autoDriveTime2, RobotMap.backward));
        // addSequential(new AutoShootLow(RobotMap.autoShootTime));
        //SmartDashboard.putNumber("2 Low", 2);

        addSequential(new IntakeToggle());
        addParallel(new AccelerateFlywheel(RobotMap.shooterVelocitySetpointOurs));
        addParallel(new AutoRunIntake(RobotMap.doNotIntakeColor, RobotMap.autoDriveTime));
        addSequential(new AutoArcadeEnocderDrive(53, RobotMap.forward, RobotMap.driveFast, 3));// was 42
        addSequential(new ResetEncoders());
        addSequential(new IntakeToggle());
        //addSequential(new Wait(3));
        addParallel(new AccelerateFlywheel(RobotMap.shooterVelocitySetpointOurs));
        addSequential(new AutoArcadeEnocderDrive(130, RobotMap.backward, RobotMap.driveFast, 4));
        //addSequential(new ResetEncoders());
        addSequential(new AutoShootLow(6));

    }
}