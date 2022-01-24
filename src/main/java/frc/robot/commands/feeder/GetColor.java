package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.feeder.*;

//import com.revrobotics.ColorSensorV3;


public class GetColor extends Command{
    boolean isBall = false;

    public GetColor() {
		// Use requires() here to declare subsystem dependencies

	}

	// Called just before this Command runs the first time
	protected void initialize() {
        
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
        Color detectedColor = Robot.m_colorSensor.getColor();
        double IR = Robot.m_colorSensor.getIR();
        
        SmartDashboard.putNumber("Red", detectedColor.red*255);
        SmartDashboard.putNumber("Green", detectedColor.green*255);
        SmartDashboard.putNumber("Blue", detectedColor.blue*255);
        SmartDashboard.putNumber("IR", IR);
        if (130>=(detectedColor.blue*255)&&(detectedColor.blue*255)>=100){
            SmartDashboard.putString("Ball Type", "BLUE");
            if (isBall == false){

                if (Feeder.checkBall1() == 0){
                    Feeder.setBall1Type(1);
                }
                else if (Feeder.checkBall2() == 0){
                    Feeder.setBall2Type(1);
                }
                isBall = true;
            }
        }
        else if (90<=detectedColor.red*255 && detectedColor.red*255<=155){
            SmartDashboard.putString("Ball Type", "RED");
            if (isBall == false){
                if (Feeder.checkBall1() == 0){
                    Feeder.setBall1Type(2);
                }
                else if (Feeder.checkBall2() == 0){
                    Feeder.setBall2Type(2);
                }
                isBall = true;
            }
        }
        else{
            SmartDashboard.putString("Ball Type", "NONE");
            isBall = false;


        }
	
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {

	}






}