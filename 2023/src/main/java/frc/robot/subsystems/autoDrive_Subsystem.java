package frc.robot.subsystems;


import frc.robot.Constants;
import frc.robot.RobotContainer;
//import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.commands.autoDrive_Command;

public class autoDrive_Subsystem extends SubsystemBase 
{



    private static WPI_VictorSPX
m_leftMotorA = new WPI_VictorSPX(Constants.m_leftMotorA),
m_leftMotorB = new WPI_VictorSPX(Constants.m_leftMotorB),
m_rightMotorA = new WPI_VictorSPX(Constants.m_rightMotorA),
m_rightMotorB = new WPI_VictorSPX(Constants.m_rightMotorB);

MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotorA, m_leftMotorB);
MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotorA, m_rightMotorB);
private  DifferentialDrive autoDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    public void setAutoDrive(double speed)
    {
       //m_leftMotorA.setInverted(true);
        //m_leftMotorB.setInverted(true);

        autoDrive.tankDrive(speed,-speed);
    }
}
