package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem
{
  /* Private Instance Variables */
  private static Intake m_Instance;
  private PeriodicIO m_PeriodicIO;

  private CANSparkMax m_IntakeMotor;

  public static Intake getInstance()
  {
    if(m_Instance == null )
    {
      m_Instance = new Intake();
    }
    return m_Instance;
  }

  private Intake()
  {
    /* Intake Motor Setup */
    m_IntakeMotor = new CANSparkMax( Constants.Intake.k_IntakeMotorCanId, MotorType.kBrushless );
    m_IntakeMotor.restoreFactoryDefaults();
    m_IntakeMotor.setIdleMode( CANSparkBase.IdleMode.kCoast );

    m_PeriodicIO = new PeriodicIO();
  }

  private static class PeriodicIO
  {
    /* Input is Desired state. */
    IntakeState state_intake = IntakeState.NONE;

    /* Intake Output: Motor Set Values */
    double intake_speed = 0.0;
  }

  public enum IntakeState
  {
    NONE,
    INTAKE_FAST,
    INTAKE_SLOW,
    EJECT
  }


/*---------------------- Intake Subsystem Functions --------------------------*/
@Override
  public void periodic()
  {
    /* Control the intake */
    m_PeriodicIO.intake_speed = intakeStateToSpeed( m_PeriodicIO.state_intake );
    SmartDashboard.putString( "Intake State:", m_PeriodicIO.state_intake.toString() );
  }

@Override
  public void writePeriodicOutputs()
  {
    m_IntakeMotor.set( m_PeriodicIO.intake_speed );
  }


@Override
  public void stop()
  {
    m_PeriodicIO.intake_speed = 0.0;
    m_PeriodicIO.state_intake = IntakeState.NONE;
  }

@Override
  public void outputTelemetry() 
  {
    SmartDashboard.putNumber("Intake State:", intakeStateToSpeed(m_PeriodicIO.state_intake));
  }

  @Override
  public void reset() 
  {
    /* nothing */
  }




/*------------------------ Public Functions ----------------------------------*/
  public double intakeStateToSpeed( IntakeState state ) 
  {
    switch( state ) 
    {
      case INTAKE_FAST:
        return Constants.Intake.k_IntakeIntakeSpeedFast;
      case INTAKE_SLOW:
        return Constants.Intake.k_IntakeIntakeSpeedSlow;
      case EJECT:
        return Constants.Intake.k_IntakeEjectSpeed;
      case NONE:
      default:
        return 0.0;
    }
  }

  public IntakeState getIntakeState() 
  {
    return m_PeriodicIO.state_intake;
  }

  /* ---------------------- Intake Helper Functions --------------------------- */
  // public void intake() 
  // {
  //   m_PeriodicIO.state_intake = IntakeState.INTAKE;
  // }

  // public void eject() 
  // {
  //   m_PeriodicIO.state_intake = IntakeState.EJECT;
  // }

  // public void stopIntake() 
  // {
  //   m_PeriodicIO.state_intake = IntakeState.NONE;
  //   m_PeriodicIO.intake_speed = 0.0;
  // }

  public void setState( IntakeState state ) 
  {
    m_PeriodicIO.state_intake = state;
  }

}
