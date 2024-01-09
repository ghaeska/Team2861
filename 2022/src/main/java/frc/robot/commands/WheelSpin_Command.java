package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class WheelSpin_Command extends CommandBase
{
    private double speed;

    public WheelSpin_Command(double speed) 
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
        Robot.wheelSpinSpeed.setSpinSpeed(speed);
    
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
