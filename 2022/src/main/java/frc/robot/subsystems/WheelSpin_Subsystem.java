package frc.robot.subsystems;


import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class WheelSpin_Subsystem extends SubsystemBase
{
    private static WPI_VictorSPX
    spinMotor = new WPI_VictorSPX(RobotMap.spinMotor);

    public void setSpinSpeed(double speed)
    {
        spinMotor.set(speed);
    }
    
}
