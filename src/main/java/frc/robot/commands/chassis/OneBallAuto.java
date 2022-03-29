package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.chassis.ModifiedArcadeDrive;
import frc.robot.commands.shooter.AccelerateFlywheel;
import frc.robot.commands.shooter.*;
import frc.robot.commands.intake.*;

public class OneBallAuto extends CommandGroup {
    public OneBallAuto() {
        
        addSequential(new AutoShootLow(RobotMap.autoOneBallLowShootTime));
        // addSequential(new AutoArcadeDrive(RobotMap.autoDriveTime2, RobotMap.forward));
        addSequential(new AutoArcadeEnocderDrive(100, RobotMap.forward,RobotMap.driveFast, 10));
    }
}