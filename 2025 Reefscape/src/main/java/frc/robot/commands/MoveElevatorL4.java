package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorSetpoint;

public class MoveElevatorL4 extends Command
{
  ElevatorSubsystem m_ElevatorSubsystem;
  boolean m_isdone;

  public MoveElevatorL4( ElevatorSubsystem subsystem )
  {
    m_ElevatorSubsystem = subsystem;
    addRequirements( m_ElevatorSubsystem );
  }

  @Override
  public void initialize()
  {
    m_isdone = false;
  }

  @Override
  public void execute()
  {
    m_ElevatorSubsystem.setElevatorSetpointCmd(ElevatorSetpoint.k_l4_up );
  }




}
