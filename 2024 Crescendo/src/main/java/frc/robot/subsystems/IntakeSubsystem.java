package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.Intake.IntakeState;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase
{

/*------------------------ Private Instance Variables ------------------------*/
  //private static Intake m_Instance;
  //private PeriodicIO m_PeriodicIO;

  private CANSparkMax m_IntakeMotor;
  private RelativeEncoder m_IntakeEncoder;

  private double m_IntakeSpeed;
  private IntakeState m_IntakeState;


  public IntakeSubsystem()
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
  }

  public void runIntake( double speed )
  {
    m_IntakeSpeed = speed;
    m_IntakeMotor.set( m_IntakeSpeed );
  }

  public void stopIntake()
  {
    m_IntakeSpeed = 0.0;
    m_IntakeState = IntakeState.STOP;
  }


  @Override
  public void periodic() 
  { 
    SmartDashboard.putNumber("Intake Set Speed:", IntakeSetSpeedFromState( m_IntakeState ));
    SmartDashboard.putString( "Intake Set State:", m_IntakeState.toString());
    SmartDashboard.putNumber("Intake Actual Speed:", m_IntakeEncoder.getVelocity());    
  }

  public enum IntakeState
  {
    STOP,         // Intake will be doing nothing
    INTAKE_FAST,  // Intake will be intaking fast
    INTAKE_SLOW,  // Intake will be intaking slow
    EJECT         // Intake will be trying to eject.
  }


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
      case STOP:
      default:
        return 0.0;
    }
  }

/***************************** Commands ************************************* */
  public Command runIntakeFastCommand()
  {
    m_IntakeState = IntakeState.INTAKE_FAST;
    return new RunCommand(()->this.runIntake( Intake.k_IntakeIntakeSpeedFast ), this );
  }

  public Command runIntakeSlowCommand()
  {
     m_IntakeState = IntakeState.INTAKE_SLOW;
    return new RunCommand(()->this.runIntake( Intake.k_IntakeIntakeSpeedSlow ), this );
  }

  public Command stopIntakeCommand()
  {
     m_IntakeState = IntakeState.STOP;
    return new RunCommand(()->this.stopIntake(), this );
  }

  public Command ejectIntakeCommand()
  {
     m_IntakeState = IntakeState.EJECT;
    return new RunCommand(()->this.runIntake( Intake.k_IntakeEjectSpeed ), this );
  }




}