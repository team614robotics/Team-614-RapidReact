package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.chassis.ModifiedArcadeDrive;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.intake.*;

public class AutoMove extends CommandGroup {
    public AutoMove() {
        //addSequential(new AutoShoot());
        addSequential(new IntakeToggle());
        addParallel(new AutoRunIntake(RobotMap.intakeSpeed, RobotMap.doNotIntakeColor));
        addSequential(new AutoArcadeDrive(RobotMap.autoDriveTime, RobotMap.forward));
        addSequential(new IntakeToggle());
        addSequential(new AutoArcadeDrive(RobotMap.autoDriveTime, RobotMap.backward));
        
        addSequential(new AutoShoot());

     

    }
}