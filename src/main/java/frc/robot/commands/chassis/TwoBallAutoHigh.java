package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.chassis.ModifiedArcadeDrive;
import frc.robot.commands.shooter.AccelerateFlywheel;
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
        addParallel(new AccelerateFlywheel(RobotMap.shooterVelocitySetpointHigh));
        addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed, RobotMap.doNotIntakeColor, RobotMap.autoDriveTime));
        addSequential(new AutoArcadeEnocderDrive(42, RobotMap.forward, RobotMap.driveFast));// was 42
        addSequential(new ResetEncoders());
        addSequential(new IntakeToggle());
        addSequential(new AutoArcadeEnocderDrive(42, RobotMap.backward, RobotMap.driveFast));
        addSequential(new ResetEncoders());
        addSequential(new AutoShootHigh(RobotMap.autoShootTime));

     

    }
}