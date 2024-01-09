package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class AirBrakeOn extends InstantCommand
{
    
    public AirBrakeOn()
    {
        
    }

    @Override
    public void initialize()
    {
        Robot.airBrake_Subsystem.AirBrakeOn();
    }


}
