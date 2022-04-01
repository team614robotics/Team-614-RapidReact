package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.chassis.RotateToAngleBackwards;
import frc.robot.commands.chassis.Wait;


public class HighShotSequence extends CommandGroup {
    public HighShotSequence() {
        // addSequential(new BlinkForATime(3));
        SmartDashboard.putNumber(" HI", 999);
        addParallel(new RotateToAngleBackwards());   
        addParallel(new RunShooterHighWithLogic());   
        // addParallel(new DeliverGoalLow());
        // addSequential(new FollowPathBackwards());
        // addSequential(new BlinkForATime(3));
        // Robot.m_drivetrain.resetPath();
        // Robot.m_drivetrain.addPoint(0, 0);
        // Robot.m_drivetrain.addPoint(-9, 0);
        // Robot.m_drivetrain.generatePath();
        // addSequential(new FollowPath());
    }
}