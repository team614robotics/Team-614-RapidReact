package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

import frc.robot.commands.chassis.ModifiedArcadeDrive;

public class AutoMove extends CommandGroup {
    public AutoMove() {
        
        addSequential(new AutoArcadeDrive());

     

    }
}