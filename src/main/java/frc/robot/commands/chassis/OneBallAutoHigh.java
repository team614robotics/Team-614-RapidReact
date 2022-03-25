package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.chassis.ModifiedArcadeDrive;
import frc.robot.commands.shooter.AccelerateFlywheel;
import frc.robot.commands.shooter.*;
import frc.robot.commands.intake.*;

public class OneBallAutoHigh extends CommandGroup {
    public OneBallAutoHigh() {
        //addSequential(new AutoShoot());
        
        // addSequential(new IntakeToggle());
        // addParallel(new AutoRunIntake(RobotMap.intakeSpeed, RobotMap.doNotIntakeColor));
        // addSequential(new AutoArcadeDrive(RobotMap.autoDriveTime, RobotMap.forward));
        // addSequential(new IntakeToggle());
        // addParallel(new AccelerateFlywheel(RobotMap.shooterVelocitySetpointOurs));
        // addSequential(new AutoArcadeDrive(RobotMap.autoDriveTime2, RobotMap.backward));
        
        
        addSequential(new AutoShootHigh(RobotMap.autoOneBallHighShootTime));
        // addSequential(new AutoArcadeDrive(RobotMap.autoDriveTime2, RobotMap.forward));
        addSequential(new AutoArcadeEnocderDrive(50, RobotMap.forward,RobotMap.driveFast));

        
        

     

    }
}