package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.Helpers;
import edu.wpi.first.math.util.Units;

public class Intake extends Subsystem
{
  //private final PIDController m_pivotPID = new PIDController( Constants.Intake.k_pivotMotorP,
  //                                                            Constants.Intake.k_pivotMotorI, 
  //                                                            Constants.Intake.k_pivotMotorD );
  //private final DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder( Constants.Intake.k_PivotEncoderId );
  //private final DigitalInput m_IntakeLimitSwitch = new DigitalInput( Constants.Intake.k_IntakeLimitSwitchId );


  /* Private Instance Variables */
  private static Intake m_Instance;
  private PeriodicIO m_PeriodicIO;

  private CANSparkMax m_IntakeMotor;
  //private CANSparkMax m_PivotMotor;

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

    /* Pivot Motor Setup */
    //m_PivotMotor = new CANSparkMax( Constants.Intake.k_PivotMotorCanId, MotorType.kBrushless );
    //m_PivotMotor.restoreFactoryDefaults();
    //m_PivotMotor.setIdleMode( CANSparkBase.IdleMode.kBrake );
    //m_PivotMotor.setSmartCurrentLimit( 10 );

    m_PeriodicIO = new PeriodicIO();
  }

  private static class PeriodicIO
  {
    /* Input is Desired state. */
    //PivotTarget pivot_target = PivotTarget.STOW;
    IntakeState state_intake = IntakeState.NONE;

    /* Intake Output: Motor Set Values */
    //double intake_pivot_voltage = 0.0;
    double intake_speed = 0.0;
  }

  //public enum PivotTarget
  //{
  //  NONE,
  //  GROUND,
  //  SOURCE,
  //  AMP,
  //  STOW
  //}

  public enum IntakeState
  {
    NONE,
    INTAKE,
    EJECT,
    //PULSE,
    //FEED_SHOOTER
  }


/* Intake Subsystem Functions */
@Override
public void periodic()
{
  /* Check to see if we have a note and need to verify position. */
  //checkAutoTasks();

  /* Pivot Control */
  //double pivot_angle = pivotTargetToAngle( m_PeriodicIO.pivot_target );
  //m_PeriodicIO.intake_pivot_voltage = m_pivotPID.calculate( getPivotAngleDegrees(), pivot_angle );

  /* Add a failsafe to check to see if encoder is connected. */
  //if( m_pivotEncoder.get() == 0.0 )
  //{
  //  /* chances are it is not connected, so disable it. */
  //  m_PeriodicIO.intake_pivot_voltage = 0.0;
  //}

  /* Control the intake */
  m_PeriodicIO.intake_speed = intakeStateToSpeed( m_PeriodicIO.state_intake );
  SmartDashboard.putString( "Intake State:", m_PeriodicIO.state_intake.toString() );
}

@Override
public void writePeriodicOutputs()
{
  //m_PivotMotor.setVoltage(m_PeriodicIO.intake_pivot_voltage );
  m_IntakeMotor.set( m_PeriodicIO.intake_speed );
}


@Override
public void stop()
{
  //m_PeriodicIO.intake_pivot_voltage = 0.0;
  m_PeriodicIO.intake_speed = 0.0;
  m_PeriodicIO.state_intake = IntakeState.NONE;
}

@Override
  public void outputTelemetry() 
  {
    SmartDashboard.putNumber("Intake speed:", intakeStateToSpeed(m_PeriodicIO.state_intake));
    //SmartDashboard.putNumber("Pivot Abs Enc (get):", m_pivotEncoder.get());
    //SmartDashboard.putNumber("Pivot Abs Enc (getAbsolutePosition):", m_pivotEncoder.getAbsolutePosition());
    //SmartDashboard.putNumber("Pivot Abs Enc (getPivotAngleDegrees):", getPivotAngleDegrees());
    //SmartDashboard.putNumber("Pivot Setpoint:", pivotTargetToAngle(m_PeriodicIO.pivot_target));

    //SmartDashboard.putNumber("Pivot Power:", m_PeriodicIO.intake_pivot_voltage);
    //SmartDashboard.putNumber("Pivot Current:", m_PivotMotor.getOutputCurrent());

    //SmartDashboard.putBoolean("Intake Limit Switch:", getIntakeHasNote());
  }

  @Override
  public void reset() 
  {
    /* nothing */
  }




/* Public Functions */

// public double pivotTargetToAngle( PivotTarget target )  
// {
//   switch( target ) 
//   {
//     case GROUND:
//       return Constants.Intake.k_PivotAngleGround;
//     case SOURCE:
//       return Constants.Intake.k_PivotAngleSource;
//     case AMP:
//       return Constants.Intake.k_PivotAngleAmp;
//     case STOW:
//       return Constants.Intake.k_PivotAngleStowed;
//     default:
//       // "Safe" default
//       return 180;
//   }
// }

public double intakeStateToSpeed( IntakeState state ) 
{
  switch( state ) 
  {
    case INTAKE:
      return Constants.Intake.k_IntakeIntakeSpeed;
    case EJECT:
      return Constants.Intake.k_IntakeEjectSpeed;
    // case PULSE:
    //   // Use the timer to pulse the intake on for a 1/16 second,
    //   // then off for a 15/16 second
    //   if( Timer.getFPGATimestamp() % 1.0 < ( 1.0 / 45.0 ) ) 
    //   {
    //     return Constants.Intake.k_IntakeIntakeSpeed;
    //   }
    //   return 0.0;
    // case FEED_SHOOTER:
    //   return Constants.Intake.k_IntakeFeedShootSpeed;
    case NONE:
    default:
      // "Safe" default
      return 0.0;
  }
}


public IntakeState getIntakeState() 
{
  return m_PeriodicIO.state_intake;
}

// public double getPivotAngleDegrees() 
// {
//   double value = m_pivotEncoder.getAbsolutePosition() 
//                  - Constants.Intake.k_PivotEncoderOffset 
//                  + 0.5;

//   return Units.rotationsToDegrees(Helpers.modRotations(value));
// }

// public boolean getIntakeHasNote() 
// {
//   // NOTE: this is intentionally inverted, because the limit switch is normally
//   // closed
//   return !m_IntakeLimitSwitch.get();
// }


/* --------------------- Pivot Helper Functions ----------------------------- */
// public void goToGround() 
// {
//   m_PeriodicIO.pivot_target = PivotTarget.GROUND;
// }

// public void goToSource() 
// {
//   m_PeriodicIO.pivot_target = PivotTarget.SOURCE;
// }

// public void goToAmp() 
// {
//   m_PeriodicIO.pivot_target = PivotTarget.SOURCE;
// }

// public void goToStow() 
// {
//   m_PeriodicIO.pivot_target = PivotTarget.STOW;
// }


/* ---------------------- Intake Helper Functions --------------------------- */
public void intake() 
{
  m_PeriodicIO.state_intake = IntakeState.INTAKE;
}

public void eject() 
{
  m_PeriodicIO.state_intake = IntakeState.EJECT;
}

// public void pulse() 
// {
//   m_PeriodicIO.state_intake = IntakeState.PULSE;
// }

// public void feedShooter() 
// {
//   m_PeriodicIO.state_intake = IntakeState.FEED_SHOOTER;
// }

public void stopIntake() 
{
  m_PeriodicIO.state_intake = IntakeState.NONE;
  m_PeriodicIO.intake_speed = 0.0;
}

public void setState( IntakeState state ) 
{
  m_PeriodicIO.state_intake = state;
}

// public void setPivotTarget( PivotTarget target ) 
// {
//   m_PeriodicIO.pivot_target = target;
// }


/* ---------------------- Private Functions --------------------------------- */
// private void checkAutoTasks() 
// {
//   // If the intake is set to GROUND, and the intake has a note, and the pivot is
//   // close to it's target
//   // Stop the intake and go to the SOURCE position
//   if( m_PeriodicIO.pivot_target == PivotTarget.GROUND && getIntakeHasNote() && isPivotAtTarget() ) 
//   {
//     m_PeriodicIO.pivot_target = PivotTarget.STOW;    
//   }
// }

// private boolean isPivotAtTarget() 
// {
//   return Math.abs(getPivotAngleDegrees() - pivotTargetToAngle(m_PeriodicIO.pivot_target)) < 5;
// }












}
