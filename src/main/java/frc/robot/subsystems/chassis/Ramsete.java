// package frc.robot.subsystems.chassis;

// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import frc.robot.Robot;
// import frc.robot.RobotMap;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Ramsete extends SubsystemBase {
//   // The motors on the left side of the drive.
//   private final MotorControllerGroup m_leftMotors =
//       new MotorControllerGroup(
//           Robot.m_drivetrain.leftMotorA,
//           Robot.m_drivetrain.leftMotorB);

//   // The motors on the right side of the drive.
//   private final MotorControllerGroup m_rightMotors =
//       new MotorControllerGroup(
//         Robot.m_drivetrain.rightMotorA,
//         Robot.m_drivetrain.rightMotorB);
        


//   // The robot's drive
//   private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

// //   // The left-side drive encoder
// //   private final Encoder m_leftEncoder =
// //       new Encoder(
// //           DriveConstants.kLeftEncoderPorts[0],
// //           DriveConstants.kLeftEncoderPorts[1],
// //           DriveConstants.kLeftEncoderReversed);

// //   // The right-side drive encoder
// //   private final Encoder m_rightEncoder =
// //       new Encoder(
// //           DriveConstants.kRightEncoderPorts[0],
// //           DriveConstants.kRightEncoderPorts[1],
// //           DriveConstants.kRightEncoderReversed);

//   // The gyro sensor
//   private final AHRS m_gyro = Robot.m_navX;

//   // Odometry class for tracking robot pose
//   private final DifferentialDriveOdometry m_odometry;

//   /** Creates a new DriveSubsystem. */
//   public Ramsete() {
//     // We need to invert one side of the drivetrain so that positive voltages
//     // result in both sides moving forward. Depending on how your robot's
//     // gearbox is constructed, you might have to invert the left side instead.
//     m_leftMotors.setInverted(true);

//     // // Sets the distance per pulse for the encoders
//     // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
//     // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

//     resetEncoders();
//     m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
//   }

//   @Override
//   public void periodic() {
//     // Update the odometry in the periodic block
//     m_odometry.update(
//         m_gyro.getRotation2d(), Robot.m_drivetrain.leftMotorA.getEncoder().getPosition(), Robot.m_drivetrain.rightMotorA.getEncoder().getPosition());
//   }

//   /**
//    * Returns the currently-estimated pose of the robot.
//    *
//    * @return The pose.
//    */
//   public Pose2d getPose() {
//     return m_odometry.getPoseMeters();
//   }

//   /**
//    * Returns the current wheel speeds of the robot.
//    *
//    * @return The current wheel speeds.
//    */
//   public DifferentialDriveWheelSpeeds getWheelSpeeds() {
//     return new DifferentialDriveWheelSpeeds(Robot.m_drivetrain.leftMotorA.getEncoder().getVelocity(), Robot.m_drivetrain.rightMotorA.getEncoder().getVelocity());
//   }

//   /**
//    * Resets the odometry to the specified pose.
//    *
//    * @param pose The pose to which to set the odometry.
//    */
//   public void resetOdometry(Pose2d pose) {
//     resetEncoders();
//     m_odometry.resetPosition(pose, m_gyro.getRotation2d());
//   }

//   /**
//    * Drives the robot using arcade controls.
//    *
//    * @param fwd the commanded forward movement
//    * @param rot the commanded rotation
//    */
//   public void arcadeDrive(double fwd, double rot) {
//     m_drive.arcadeDrive(fwd, rot);
//   }

//   /**
//    * Controls the left and right sides of the drive directly with voltages.
//    *
//    * @param leftVolts the commanded left output
//    * @param rightVolts the commanded right output
//    */
//   public void tankDriveVolts(double leftVolts, double rightVolts) {
//     SmartDashboard.putNumber("Ramsete", 2);
//     m_leftMotors.setVoltage(leftVolts);
//     m_rightMotors.setVoltage(rightVolts);
//     m_drive.feed();
//   }

//   /** Resets the drive encoders to currently read a position of 0. */
//   public void resetEncoders() {
//     //SmartDashboard.putNumber("Left Encoder Values", Robot.m_drivetrain.leftMotorA.().getPosition());
//     Robot.m_drivetrain.leftMotorA.getEncoder().setPosition(0);
//     Robot.m_drivetrain.leftMotorB.getEncoder().setPosition(0);
//     //SmartDashboard.putNumber("Right Encoder Values", Robot.m_drivetrain.rightMotorA.getEncoder().getPosition());
//     Robot.m_drivetrain.rightMotorA.getEncoder().setPosition(0);
//     Robot.m_drivetrain.rightMotorB.getEncoder().setPosition(0);
//   }

//   /**
//    * Gets the average distance of the two encoders.
//    *
//    * @return the average of the two encoder readings
//    */
//   public double getAverageEncoderDistance() {
//     return (Robot.m_drivetrain.leftMotorA.getEncoder().getPosition() + Robot.m_drivetrain.rightMotorA.getEncoder().getPosition()) / 2.0;
//   }

// //   /**
// //    * Gets the left drive encoder.
// //    *
// //    * @return the left drive encoder
// //    */
// //   public Encoder getLeftEncoder() {
// //     return m_leftEncoder;
// //   }

// //   /**
// //    * Gets the right drive encoder.
// //    *
// //    * @return the right drive encoder
// //    */
// //   public Encoder getRightEncoder() {
// //     return m_rightEncoder;
// //   }

//   /**
//    * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
//    *
//    * @param maxOutput the maximum output to which the drive will be constrained
//    */
//   public void setMaxOutput(double maxOutput) {
//     m_drive.setMaxOutput(maxOutput);
//   }

//   /** Zeroes the heading of the robot. */
//   public void zeroHeading() {
//     m_gyro.reset();
//   }

//   /**
//    * Returns the heading of the robot.
//    *
//    * @return the robot's heading in degrees, from -180 to 180
//    */
//   public double getHeading() {
//     return m_gyro.getRotation2d().getDegrees();
//   }

//   /**
//    * Returns the turn rate of the robot.
//    *
//    * @return The turn rate of the robot, in degrees per second
//    */
//   public double getTurnRate() {
//     return -m_gyro.getRate();
//   }

  
// }