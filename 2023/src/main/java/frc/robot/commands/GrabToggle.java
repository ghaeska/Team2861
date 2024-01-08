package frc.robot.commands;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Grab_Subsystem;

public class GrabToggle extends InstantCommand
{
    
    public GrabToggle()
    {
        
    }

    @Override
    public void initialize()
    {
        Robot.Grab_Subsystem.grabToggle();
    }


}
