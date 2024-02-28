package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.Intake;
import frc.robot.Constants;
//import frc.robot.Constants.Index;
import frc.robot.Constants.Shooter;
//import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeIndexSubsystem;

public class PrepareShootNote extends Command
{
  IntakeIndexSubsystem m_IntakeIndexSubsystem;
  ShooterSubsystem m_ShooterSubsystem;
  boolean m_isDone;

  public PrepareShootNote( IntakeIndexSubsystem i_Subsystem, ShooterSubsystem s_Subsystem )
  {
    m_IntakeIndexSubsystem = i_Subsystem;
    m_ShooterSubsystem = s_Subsystem;
    addRequirements(m_IntakeIndexSubsystem, m_ShooterSubsystem );
  }

  @Override
  public void initialize() 
  {
    m_isDone = false;
  }

  @Override
  public void execute() 
  {
    System.out.print( "Starting up the shooter Motor \r\n");
    m_ShooterSubsystem.runShooter( Shooter.k_ShooterSpeed_Speaker );

    while (m_ShooterSubsystem.m_TopShooterEncoder.getVelocity() >= 2000) 
    {
      System.out.print( "Shooter is at speed.\r\n");
    
      if( m_IntakeIndexSubsystem.isNoteInIndex() )
      {
        System.out.print("Note is in the Intake, Ready to shoot. \r\n");
        //m_IntakeIndexSubsystem.runIndex( Constants.Index.k_IndexForwardSpeed );
        //m_isDone = true;
      }
      else
      {
        //m_isDone = true;
      }

      if( m_IntakeIndexSubsystem.isNoteInIndex() == false )
      {
        System.out.print( "about to end. \r\n");
        m_isDone = true;
      }
    
    
    
    }
    
  }

  @Override
  public void end(boolean interrupted) 
  {
    // Turn off the Index and Shooter
    m_IntakeIndexSubsystem.stopIndex();
    m_ShooterSubsystem.stopShooter();
  }

  @Override
  public boolean isFinished() 
  {
    return m_isDone;
  }






}


