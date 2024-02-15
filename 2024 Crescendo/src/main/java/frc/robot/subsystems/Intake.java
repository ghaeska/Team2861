package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem
{
  /*
  ** This file uses a state machine to know how the intake should be operating.
  ** Call setState() with a state that you would like to get to and it will 
  ** put the intake into that state. 
  */

/*------------------------ Private Instance Variables ------------------------*/
  private static Intake m_Instance;
  private PeriodicIO m_PeriodicIO;

  private CANSparkMax m_IntakeMotor;
  private RelativeEncoder m_IntakeEncoder;

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
    /* Setup the Motor Controllers */
    m_IntakeMotor = new CANSparkMax( Constants.Intake.k_IntakeMotorCanId, MotorType.kBrushless );

    /* Restore them to defaults */
    m_IntakeMotor.restoreFactoryDefaults();

    /* Set the motors Idle Mode */
    m_IntakeMotor.setIdleMode( CANSparkBase.IdleMode.kCoast );

    /* Set the current limits on the Motors */
    m_IntakeMotor.setSmartCurrentLimit( 20 );

    /* Setup the Motor Encoders */
    m_IntakeEncoder = m_IntakeMotor.getEncoder();

    /* Burn the new settings into the flash. */
    m_IntakeMotor.burnFlash();

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
    NONE,         // Intake will be doing nothing
    INTAKE_FAST,  // Intake will be intaking fast
    INTAKE_SLOW,  // Intake will be intaking slow
    EJECT         // Intake will be trying to eject.
  }

/*---------------------- Intake Subsystem Functions --------------------------*/
@Override
  public void periodic()
  {
    /* Control the intake */
    m_PeriodicIO.intake_speed = IntakeSetSpeedFromState( m_PeriodicIO.state_intake );   
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
    SmartDashboard.putNumber("Intake Set Speed:", IntakeSetSpeedFromState(m_PeriodicIO.state_intake));
    SmartDashboard.putString( "Intake Set State:", m_PeriodicIO.state_intake.toString());
    SmartDashboard.putNumber("Intake Actual Speed:", m_IntakeEncoder.getVelocity());
  }

  @Override
  public void reset() 
  {
    /* nothing */
  }

/*------------------------ Public Functions ----------------------------------*/
  public double IntakeSetSpeedFromState( IntakeState state ) 
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
  public void setState( IntakeState state ) 
  {
    m_PeriodicIO.state_intake = state;
  }

}
