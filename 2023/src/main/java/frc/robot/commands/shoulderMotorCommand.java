package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class shoulderMotorCommand extends CommandBase
{
    
    private double speed;

    public shoulderMotorCommand (double speed) 
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
        Robot.shoulderMotorSpeed.setShoulderSpeed(speed);
    }

    //@Override
    public boolean isFinished()
    {
        return true;

    }
    //@Override
    protected void end(){

    }

    //@Override
    protected void interrupted(){
    }

 }

