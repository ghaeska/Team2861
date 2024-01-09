
package frc.robot.subsystems;


import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Claw1_Subsystem extends SubsystemBase 
{
private static WPI_VictorSPX 
  clawMotor = new WPI_VictorSPX(Constants.clawMotor);
 

public void setClawSpeed(double speed)
{
    clawMotor.set(speed);
   
}
}
