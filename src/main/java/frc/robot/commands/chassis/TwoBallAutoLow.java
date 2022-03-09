package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.chassis.ModifiedArcadeDrive;
import frc.robot.commands.shooter.AccelerateFlywheel;
import frc.robot.commands.shooter.*;
import frc.robot.commands.intake.*;

public class TwoBallAutoLow extends CommandGroup {
    int accelerationSpeed;
    public TwoBallAutoLow() {
        //addSequential(new AutoShoot());
        accelerationSpeed = RobotMap.shooterVelocitySetpointOurs;
        addSequential(new IntakeToggle());
        addParallel(new AutoRunIntake(RobotMap.intakeSpeed, RobotMap.doNotIntakeColor));
        addSequential(new AutoArcadeDrive(RobotMap.autoDriveTime, RobotMap.forward));
        addSequential(new IntakeToggle());
        addParallel(new AccelerateFlywheel(accelerationSpeed));
        addSequential(new AutoArcadeDrive(RobotMap.autoDriveTime2, RobotMap.backward));
        addSequential(new AutoShootLow());
        

     

    }
}