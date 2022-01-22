package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

import frc.robot.commands.chassis.ModifiedArcadeDrive;
import frc.robot.commands.shooter.AutoShoot;

public class AutoMove extends CommandGroup {
    public AutoMove() {
        addSequential(new AutoShoot());
        addSequential(new AutoArcadeDrive());

     

    }
}