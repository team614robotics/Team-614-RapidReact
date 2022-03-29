// package frc.robot.commands.chassis;

// import edu.wpi.first.wpilibj.command.CommandGroup;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Robot;
// import frc.robot.RobotMap;
// import frc.robot.commands.chassis.ModifiedArcadeDrive;
// import frc.robot.commands.shooter.*;
// import frc.robot.commands.intake.*;

// public class ThreeBallHighAuto extends CommandGroup {
//   public ThreeBallHighAuto() {

//     // addSequential(new IntakeToggle());
//     // addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed,
//     // RobotMap.doNotIntakeColor,RobotMap.autoDriveTime));
//     // addSequential(new AutoArcadeEnocderDrive(38, RobotMap.forward));//was 42
//     // addSequential(new AutoArcadeRotate(Robot.m_navX, 120,
//     // RobotMap.forward));//was 177
//     // addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed,
//     // RobotMap.doNotIntakeColor,4));
//     // addSequential(new AutoArcadeEnocderDrive(110, RobotMap.forward));
//     // addParallel(new AccelerateFlywheel(RobotMap.shooterVelocitySetpointHigh));
//     // addSequential(new AutoArcadeRotate(Robot.m_navX, 76, RobotMap.backward));
//     // addSequential(new IntakeToggle());


//     addSequential(new IntakeToggle());
//     addParallel(new AccelerateFlywheel(RobotMap.shooterVelocitySetpointHigh));
//     addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed, RobotMap.doNotIntakeColor, RobotMap.autoDriveTime));
//     .//addSequential(new AutoArcadeEnocderDrive(38, RobotMap.forward, RobotMap.driveFast));// was 42
//     addSequential(new ResetEncoders());
//     addSequential(new IntakeToggle());
//     //addSequential(new AutoArcadeEnocderDrive(38, RobotMap.backward, RobotMap.driveFast));
//     addSequential(new ResetEncoders());
//     addSequential(new AutoShootHigh(RobotMap.autoShootTime));
//     addSequential(new AutoArcadeRotate(Robot.m_navX, 87, RobotMap.clockwise, 1));
//     addSequential(new IntakeToggle());
//     addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed, RobotMap.doNotIntakeColor, 3));
//     addSequential(new AutoArcadeEnocderDrive(94, RobotMap.forward, RobotMap.driveFast));// was 42
//     addSequential(new ResetEncoders());
//     addSequential(new IntakeToggle());
//     addSequential(new AutoArcadeRotate(Robot.m_navX, 31, RobotMap.counterClockwise, .5));
//     addSequential(new AutoArcadeEnocderDrive(60, RobotMap.backward, RobotMap.driveFast));
//     addSequential(new AutoShootHigh(RobotMap.autoShootTime));
//     addSequential(new AutoArcadeRotate(Robot.m_navX, 10, RobotMap.clockwise, .5));
//     addSequential(new IntakeToggle());
//     addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed, RobotMap.doNotIntakeColor, 3));
//     addSequential(new AutoArcadeEnocderDrive(185, RobotMap.forward, -.9));
//     addSequential(new ResetEncoders());
//     addSequential(new IntakeToggle());
//     addSequential(new AutoArcadeEnocderDrive(185, RobotMap.backward, -.9));
//     //addSequential(new AutoShootHigh(RobotMap.autoShootTime));

//     // addSequential(new AutoArcadeRotate(Robot.m_navX, 120,
//     // RobotMap.forward));//was 177
//     // addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed,
//     // RobotMap.doNotIntakeColor,4));
//     // addSequential(new AutoArcadeEnocderDrive(110, RobotMap.forward));
//     // addParallel(new AccelerateFlywheel(RobotMap.shooterVelocitySetpointHigh));
//     // addSequential(new AutoArcadeRotate(Robot.m_navX, 76, RobotMap.backward));
//     // addSequential(new IntakeToggle());

//   }
// }