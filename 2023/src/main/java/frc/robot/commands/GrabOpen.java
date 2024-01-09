package frc.robot.commands;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Grab_Subsystem;

public class GrabOpen extends InstantCommand
{
    
    public GrabOpen()
    {
        
    }

    @Override
    public void initialize()
    {
        Robot.Grab_Subsystem.grabClose();
    }


}
