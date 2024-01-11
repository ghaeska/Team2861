package frc.robot.subsystems;


import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
public class elbowMotorSub extends SubsystemBase 
{
private static WPI_VictorSPX 
  elbowMotorA = new WPI_VictorSPX(Constants.elbowMotorA);
  //elbowMotorB = new WPI_VictorSPX(Constants.elbowMotorB);
DigitalInput baseLimit = new DigitalInput(Constants.baseSwitchBack);
private double elbowSpeed = 0;
public void setElbowSpeed(double speed)
{
  elbowSpeed = speed;
    elbowMotorA.set(speed);
    //elbowMotorB.set(speed);
    
}

public boolean backLimitValue() {return baseLimit.get();}

public void periodic()
{
/* 
  super.periodic();

if(!backLimitValue())
  {
    elbowMotorA.set(elbowSpeed);
  }
  else if(backLimitValue() && elbowSpeed >= 0)
  {
    elbowMotorA.set(elbowSpeed);
  }
else 
{
  elbowMotorA.set(0);
}



*/

}

}
