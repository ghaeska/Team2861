package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class IntakeTask extends Task 
{
  private Intake m_intake;
  private IntakeState m_intakeState;

  public IntakeTask( IntakeState intakeState ) 
  {
    m_intake = Intake.getInstance();
    m_intakeState = intakeState;
  }

  @Override
  public void start() 
  {
    m_intake.setState( m_intakeState );
  }

  @Override
  public void update() 
  {
  }

  @Override
  public boolean isFinished() 
  {
    return true;
  }
}
