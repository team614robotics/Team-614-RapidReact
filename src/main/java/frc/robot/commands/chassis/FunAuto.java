package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.chassis.ModifiedArcadeDrive;
import frc.robot.commands.feeder.RunFeeder;
import frc.robot.commands.shooter.*;
import frc.robot.commands.intake.*;


public class FunAuto extends CommandGroup{
    public FunAuto() {
        
        addSequential(new AutoArcadeRotate(Robot.m_navX, 360, RobotMap.counterClockwise, 15));
    }
}
