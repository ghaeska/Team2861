package frc.robot.subsystems;


import frc.robot.RobotMap;
//import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;


public class Lift_Subsystem extends SubsystemBase

{
 private static WPI_VictorSPX
 liftMotor = new WPI_VictorSPX(RobotMap.liftMotor);

 DigitalInput topLimitSwitch = new DigitalInput(RobotMap.topLiftLimitSwitch),
    botLimitSwitch = new DigitalInput(RobotMap.botLiftLimitSwitch);

private double liftSpeed = 0;

public void setLiftSpeed(double speed)
{
    liftSpeed = speed;
    liftMotor.set(speed);

}
public boolean topLimitValue() {return !topLimitSwitch.get();}
public boolean botLimitValue() {return !botLimitSwitch.get();}
public void periodic() 
{
    super.periodic();

    if(!topLimitValue()  && liftSpeed>0)
    {
       //Do Nothing
       liftMotor.set(liftSpeed);
       
    }
    else if(!botLimitValue() && liftSpeed<0)
    {
        //only allow negative movement
        liftMotor.set(liftSpeed);
       
    }
    else
     {
         liftMotor.set(0);
         
    }

    //  if(botLimitValue() && liftSpeed>0)
    // {
    //     //only allow positive movement
    //     liftMotor.set(liftSpeed);
    // }
    //  else if(botLimitValue() && liftSpeed<0)
    //  {
    //      /* Do Nothing */
    //  }
    // else
    // {
    //     // Allow either direction
    //     liftMotor.set(0);
    
    // }
    
}

}
