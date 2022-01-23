package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.RobotMap;


public class Feeder extends Subsystem
{
    public CANSparkMax feederMotor;
    public static int ball1Type;
    public static int ball2Type;
    //1 = BLUE, 2 = RED

    public Feeder(){
        feederMotor = new CANSparkMax(RobotMap.feederMotorPort, MotorType.kBrushless);
        ball1Type = 0;
        ball2Type = 0;
    } 
    
    @Override
    public void initDefaultCommand() {
        // TODO Auto-generated method stub
        
    } {
		// Use requires() here to declare subsystem dependencies
	}

    public static void setBall1Type(int type) {
        ball1Type = type;
    }
    public static void setBall2Type(int type) {
        ball2Type = type;
    }
    public static int checkBall1() {
        return ball1Type;
    }
    public static int checkBall2() {
        return ball2Type;
    }
    

	
}
