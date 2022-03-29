package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.chassis.ModifiedArcadeDrive;
import frc.robot.commands.feeder.RunFeeder;
import frc.robot.commands.shooter.*;
import frc.robot.commands.intake.*;

public class TwoBallAutoHigh extends CommandGroup {
    int accelerationSpeed;
    public TwoBallAutoHigh() {
        //addSequential(new AutoShoot());
        // accelerationSpeed = RobotMap.shooterVelocitySetpointHigh;
        // //SmartDashboard.putNumber("2 High", 1);
        // addSequential(new IntakeToggle());
        // addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed, RobotMap.doNotIntakeColor, RobotMap.autoDriveTime));
        // addSequential(new AutoArcadeDrive(RobotMap.autoDriveTime, RobotMap.forward));
        // addSequential(new IntakeToggle());
        // addParallel(new AccelerateFlywheel(accelerationSpeed));
        // addSequential(new AutoArcadeDriveHighShot(RobotMap.backward));
        
        // addSequential(new AutoShootLow(1));
        //SmartDashboard.putNumber("2 High", 2);

        addSequential(new IntakeToggle());
        addParallel(new AccelerateFlywheel(RobotMap.shooterVelocitySetpointOurs));
        addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed, RobotMap.doNotIntakeColor, RobotMap.autoDriveTime));
        addSequential(new AutoArcadeEnocderDrive(42, RobotMap.forward, RobotMap.driveFast, 3));// was 42
        addSequential(new ResetEncoders());
        addSequential(new IntakeToggle());
        //addSequential(new Wait(4));
        addParallel(new AccelerateFlywheel(RobotMap.shooterVelocitySetpointOurs));
        addSequential(new AutoArcadeEnocderDrive(130, RobotMap.backward, RobotMap.driveFast, 5));
        addSequential(new ResetEncoders());
        addSequential(new AutoArcadeEnocderDrive(40, RobotMap.forward, RobotMap.driveFast, 2));
        addSequential(new AutoShootHigh(5));

     

    }
}