package frc.robot.subsystems;


import frc.robot.Constants;
import frc.robot.RobotContainer;
//import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.commands.autoArm_Command;

public class autoArm_Subsystem extends SubsystemBase 
{


private static WPI_VictorSPX
shoulderMotorA = new WPI_VictorSPX(Constants.shoulderMotorA),
shoulderMotorB = new WPI_VictorSPX(Constants.shoulderMotorB),
clawMotor = new WPI_VictorSPX(Constants.clawMotor);

MotorControllerGroup m_ShoulderMotors = new MotorControllerGroup(shoulderMotorA, shoulderMotorB);
MotorControllerGroup m_rightMotors = new MotorControllerGroup(clawMotor);
private  DifferentialDrive autoArm = new DifferentialDrive(clawMotor, m_ShoulderMotors);

    public void setAutoArm(double claw, double shoulder )
    {
       //m_leftMotorA.setInverted(true);
        //m_leftMotorB.setInverted(true);

        autoArm.tankDrive(claw, shoulder);
    }
}
