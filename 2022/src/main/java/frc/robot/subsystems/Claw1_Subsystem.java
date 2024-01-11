
package frc.robot.subsystems;


import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Claw1_Subsystem extends SubsystemBase 
{
private static WPI_VictorSPX 
  clawMotor1 = new WPI_VictorSPX(RobotMap.clawMotor1);
  //clawMotor2 = new WPI_VictorSPX(RobotMap.clawMotor2),
  //clawMotor3 = new WPI_VictorSPX(RobotMap.clawMotor3);

public void setClawSpeed(double speed)
{
    clawMotor1.set(speed);
    //clawMotor2.set(speed);
    //clawMotor3.set(speed);
}
}
