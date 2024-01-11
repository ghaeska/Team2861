package frc.robot.subsystems;


import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class shoulderMotorSub extends SubsystemBase 
{
private static WPI_VictorSPX 
  shoulderMotorA = new WPI_VictorSPX(Constants.shoulderMotorA),
  shoulderMotorB = new WPI_VictorSPX(Constants.shoulderMotorB);

//MotorControllerGroup shoulderMotors = new MotorControllerGroup(shoulderMotorA, shoulderMotorB);


public void setShoulderSpeed(double speed)
{
    shoulderMotorA.set(speed);
    shoulderMotorB.set(speed);
  
}
}
