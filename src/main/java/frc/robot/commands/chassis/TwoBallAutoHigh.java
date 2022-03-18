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
        accelerationSpeed = RobotMap.shooterVelocitySetpointHigh;
        //SmartDashboard.putNumber("2 High", 1);
        addSequential(new IntakeToggle());
        addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed, RobotMap.doNotIntakeColor));
        addSequential(new AutoArcadeDrive(RobotMap.autoDriveTime, RobotMap.forward));
        addSequential(new IntakeToggle());
        addParallel(new AccelerateFlywheel(accelerationSpeed));
        addSequential(new AutoArcadeDriveHighShot(RobotMap.backward));
        
        addSequential(new AutoShootHigh());
        //SmartDashboard.putNumber("2 High", 2);

     

    }
}