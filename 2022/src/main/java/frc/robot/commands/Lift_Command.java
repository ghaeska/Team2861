package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Lift_Command extends CommandBase
{
 
    
    
    private double speed;

    public Lift_Command(double speed) 
    {
        this.speed = speed;
    }
        
    //@Override
    public void inititalize() 
    {
    
    }
    
    //@Override
    
    public void execute()
    {
     
        Robot.liftSub.setLiftSpeed(speed);
        
    }
    
    //@Override
    public boolean isFinished()
    {
        return true;
    
    }
    //@Override
    protected void end()
    {
    
    }
    
    //@Override
    protected void interrupted()
    {

    }
    



}
