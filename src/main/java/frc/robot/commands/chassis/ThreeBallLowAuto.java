// package frc.robot.commands.chassis;

// import edu.wpi.first.wpilibj.command.CommandGroup;
// import frc.robot.Robot;
// import frc.robot.RobotMap;
// import frc.robot.commands.chassis.ModifiedArcadeDrive;
// import frc.robot.commands.shooter.*;
// import frc.robot.commands.intake.*;

// public class ThreeBallLowAuto extends CommandGroup {
//   public ThreeBallLowAuto() {
   
//     addSequential(new IntakeToggle());
//     addParallel(new AccelerateFlywheel(RobotMap.shooterVelocitySetpointOurs));
//     addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed, RobotMap.doNotIntakeColor, RobotMap.autoDriveTime));
//     addSequential(new AutoArcadeEnocderDrive(38, RobotMap.forward, RobotMap.driveFast));// was 42
//     addSequential(new ResetEncoders());
//     addSequential(new IntakeToggle());
//     addSequential(new AutoArcadeEnocderDrive(110, RobotMap.backward, RobotMap.driveFast));
//     addSequential(new ResetEncoders());
//     addSequential(new AutoShootLow(RobotMap.autoThreeBallLowShootTime));
//     addSequential(new AutoArcadeRotate(Robot.m_navX, 39, RobotMap.clockwise,1));
//     addSequential(new IntakeToggle());
//     addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed, RobotMap.doNotIntakeColor, 2.5));
//     addSequential(new AutoArcadeEnocderDrive(100, RobotMap.forward, RobotMap.driveFast));
//     addSequential(new ResetEncoders());
//     addSequential(new IntakeToggle());
//     addSequential(new AutoArcadeEnocderDrive(100, RobotMap.backward, RobotMap.driveFast));
//     addSequential(new AutoShootLow(RobotMap.autoThreeBallLowShootTime));

//     // addSequential(new IntakeToggle());
//     // addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed, RobotMap.doNotIntakeColor, 3));
//     // addSequential(new AutoArcadeEnocderDrive(94, RobotMap.forward));// was 42
//     // addSequential(new ResetEncoders());
//     // addSequential(new IntakeToggle());
//     // addSequential(new AutoArcadeRotate(Robot.m_navX, 31, RobotMap.counterClockwise,.5));
//     // addSequential(new AutoArcadeEnocderDrive(60, RobotMap.backward));
//     // addSequential(new AutoShootHigh());
//     // addSequential(new AutoArcadeRotate(Robot.m_navX, 31, RobotMap.counterClockwise,.5));
//     // addSequential(new AutoArcadeEnocderDrive(185, RobotMap.forward));
//     // addSequential(new IntakeToggle());
//     // addParallel(new AutoRunIntake(RobotMap.autoIntakeSpeed, RobotMap.doNotIntakeColor, 3));



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