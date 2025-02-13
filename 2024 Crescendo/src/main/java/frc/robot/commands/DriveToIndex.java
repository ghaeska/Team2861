package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.Constants;
import frc.robot.Constants.Index;
//import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeIndexSubsystem;

public class DriveToIndex extends Command
{
  IntakeIndexSubsystem m_IntakeIndexSubsystem;
  boolean m_isDone;

  public DriveToIndex( IntakeIndexSubsystem subsytem )
  {
    m_IntakeIndexSubsystem = subsytem;
    addRequirements( m_IntakeIndexSubsystem );
  }

  @Override
  public void initialize() 
  {
    m_isDone = false;
  }

  @Override
  public void execute() 
  {

    // Turn on the Intake
    m_IntakeIndexSubsystem.runIntake(Constants.Intake.k_IntakeIntakeSpeedSlow );
    m_IntakeIndexSubsystem.runIndex( Constants.Index.k_IndexForwardSpeed );
  
    if( m_IntakeIndexSubsystem.isNoteInIndex() )
    {
      System.out.print(" Note is in the Index \r\n");
      m_IntakeIndexSubsystem.stopIndex();
      m_IntakeIndexSubsystem.stopIntake();
      m_isDone = true;
    }

  }

  @Override
  public void end(boolean interrupted) 
  {
    // Turn off the Index and Intake
    m_IntakeIndexSubsystem.stopIndex();
    m_IntakeIndexSubsystem.stopIntake();
  }

  @Override
  public boolean isFinished() 
  {
    return m_isDone;
  }
  
}
