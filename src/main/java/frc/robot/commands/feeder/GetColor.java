package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.feeder.*;
import edu.wpi.first.wpilibj.*;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;



//import com.revrobotics.ColorSensorV3;

public class GetColor extends Command {
    boolean isBall = false; // j if there is a ball in system true
    boolean isRecorded = false;// j what is the point of knowing if the ball is recorded
    Timer timer;

    public GetColor() {
        // Use requires() here to declare subsystem dependencies
        timer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        isBall = false;
        isRecorded = false;
        Robot.m_feeder.feederMotor.set(RobotMap.initalizeFeederMotor);
        timer.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Color detectedColor = Robot.m_colorSensor.getColor();
        int proximity = Robot.m_colorSensor.getProximity();
        SmartDashboard.putNumber("Proximity", proximity);
        double IR = Robot.m_colorSensor.getIR();

        SmartDashboard.putNumber("Red", detectedColor.red * RobotMap.colorMultiplier);
        SmartDashboard.putNumber("Green", detectedColor.green * RobotMap.colorMultiplier);
        SmartDashboard.putNumber("Blue", detectedColor.blue * RobotMap.colorMultiplier);
        SmartDashboard.putNumber("IR", IR);
        if (proximity > RobotMap.ballProximity) {
            isBall = true;
            Feeder.setBall(true);
        } else {
            Feeder.setBall(false);
            SmartDashboard.putString("Ball Type", "NONE");
            OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.rumbleOff);
            OI.driverController.setRumble(GenericHID.RumbleType.kLeftRumble, RobotMap.rumbleOff);
            Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
            isRecorded = false;
            isBall = false;

        }
        if (detectedColor.red > detectedColor.blue) {
            if (isBall) {
                SmartDashboard.putString("Ball Type", "RED");

            }
            if (!isRecorded && isBall) {
                if (Feeder.checkBall1() == RobotMap.noBall) { // j checks for first ball if not puts red ball into ball1
                    Feeder.setBall1Type(RobotMap.RedAlliance);
                    // Rumbles more if the ball is ours
                    if (Robot.allianceColor == RobotMap.RedAlliance) { // j allianceColor = 1 if blue = 2 if red
                        OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.rumbleOurBall);
                    } else if (Robot.allianceColor == RobotMap.BlueAlliance) {
                        OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.rumbleOpposingBall);
                    } else {
                        OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.neutralRumble);// j
                                                                                                                  // allaianceColor
                                                                                                                  // never
                                                                                                                  // !=
                                                                                                                  // 1
                                                                                                                  // or
                                                                                                                  // 2?
                    }
                    timer.reset();
                    timer.start();
                    //Robot.m_feeder.feederMotor.set(RobotMap.feederSpeed);
                    for (var i = 0; i < Robot.m_ledBuffer.getLength() / 2; i++) {// j sets the first half of the LEDs to
                                                                                 // red
                        // Sets the specified LED to the RGB values for red
                        Robot.m_ledBuffer.setRGB(i, RobotMap.redRValue, RobotMap.redGValue, RobotMap.redBValue);
                    }

                } else if (Feeder.checkBall2() == RobotMap.noBall) {// j checks for second ball if not puts red ball
                                                                    // into ball2
                    Feeder.setBall2Type(RobotMap.RedAlliance);
                    if (Robot.allianceColor == RobotMap.RedAlliance) {
                        OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.rumbleOurBall);
                    } else if (Robot.allianceColor == RobotMap.BlueAlliance) {
                        OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.rumbleOpposingBall);
                    } else {
                        OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.neutralRumble);
                    }
                    // Robot.m_feeder.feederMotor.set(RobotMap.feederSpeed);
                    for (var i = Robot.m_ledBuffer.getLength() / 2; i < Robot.m_ledBuffer.getLength(); i++) {// j sets
                                                                                                             // the
                                                                                                             // second
                                                                                                             // half of
                                                                                                             // the LEDs
                                                                                                             // to red
                        // Sets the specified LED to the RGB values for red
                        Robot.m_ledBuffer.setRGB(i, RobotMap.redRValue, RobotMap.redGValue, RobotMap.redBValue);
                    }
                }
                isRecorded = true;
            }
        } else if (detectedColor.blue > detectedColor.red) {// j why is this here if is not red it has to be blue
            if (isBall) { // j does the same thing but for if the ball is blue
                SmartDashboard.putString("Ball Type", "BLUE");

            }
            if (!isRecorded && isBall) {
                if (Feeder.checkBall1() == RobotMap.noBall) {
                    Feeder.setBall1Type(RobotMap.BlueAlliance);
                    // Rumbles more if the ball is ours
                    if (Robot.allianceColor == RobotMap.BlueAlliance) {
                        OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.rumbleOurBall);
                    } else if (Robot.allianceColor == RobotMap.RedAlliance) {
                        OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.rumbleOurBall);
                    } else {
                        OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.neutralRumble);
                    }
                    timer.reset();
                    timer.start();
                    //Robot.m_feeder.feederMotor.set(RobotMap.feederSpeed);
                    for (var i = 0; i < Robot.m_ledBuffer.getLength() / 2; i++) {
                        // Sets the specified LED to the RGB values for red
                        Robot.m_ledBuffer.setRGB(i, RobotMap.blueRValue, RobotMap.blueGValue, RobotMap.blueBValue);
                    }
                } else if (Feeder.checkBall2() == RobotMap.noBall) {
                    Feeder.setBall2Type(RobotMap.BlueAlliance);
                    if (Robot.allianceColor == RobotMap.BlueAlliance) {
                        OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.rumbleOurBall);
                    } else if (Robot.allianceColor == RobotMap.RedAlliance) {
                        OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.rumbleOurBall);
                    } else {
                        OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.neutralRumble);
                    }
                    // Robot.m_feeder.feederMotor.set(RobotMap.feederSpeed);
                    for (var i = Robot.m_ledBuffer.getLength() / 2; i < Robot.m_ledBuffer.getLength(); i++) {
                        // Sets the specified LED to the RGB values for red
                        Robot.m_ledBuffer.setRGB(i, RobotMap.blueRValue, RobotMap.blueGValue, RobotMap.blueBValue);
                    }
                }
                isRecorded = true;
            }
        }
        if (timer.get()<0.75 && !(Feeder.checkBall1() == RobotMap.noBall)){
            Robot.m_feeder.feederMotor.set(RobotMap.feederSpeed);
        }
        else if (timer.get() > 0.75){
            Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);
            timer.stop();
            
        }

        // if (130>=(detectedColor.blue*255)&&(detectedColor.blue*255)>=100){
        // SmartDashboard.putString("Ball Type", "BLUE");
        // if (isBall == false){

        // if (Feeder.checkBall1() == 0){
        // Feeder.setBall1Type(1);
        // }
        // else if (Feeder.checkBall2() == 0){
        // Feeder.setBall2Type(1);
        // }
        // isBall = true;
        // }
        // }
        // else if (90<=detectedColor.red*255 && detectedColor.red*255<=155){
        // SmartDashboard.putString("Ball Type", "RED");
        // if (isBall == false){
        // if (Feeder.checkBall1() == 0){
        // Feeder.setBall1Type(2);
        // }
        // else if (Feeder.checkBall2() == 0){
        // Feeder.setBall2Type(2);
        // }
        // isBall = true;
        // }
        // }
        // else{
        // SmartDashboard.putString("Ball Type", "NONE");
        // isBall = false;

        // }

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        SmartDashboard.putNumber("ColorEnd", 1);
        OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.rumbleOff);
        OI.driverController.setRumble(GenericHID.RumbleType.kLeftRumble, RobotMap.rumbleOff);
        Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        SmartDashboard.putNumber("ColorEnd", 2);
        OI.driverController.setRumble(GenericHID.RumbleType.kRightRumble, RobotMap.rumbleOff);
        OI.driverController.setRumble(GenericHID.RumbleType.kLeftRumble, RobotMap.rumbleOff);
        Robot.m_feeder.feederMotor.set(RobotMap.turnOffFeederMotor);

    }

}