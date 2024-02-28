package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.Intake;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj2.command.button;

public class IntakeSubsystem extends SubsystemBase
{

/*------------------------ Private Instance Variables ------------------------*/
  private CANSparkMax m_IntakeMotor;
  private RelativeEncoder m_IntakeEncoder;

  public DigitalInput m_IntakeSensor;
  public boolean m_NoteInIntake = false;

  private double m_IntakeSpeed;

  public IntakeSubsystem()
  {
    /* Setup the Motor Controllers */
    m_IntakeMotor = new CANSparkMax( Constants.Intake.k_IntakeMotorCanId, MotorType.kBrushless );

    /* Restore them to defaults */
    m_IntakeMotor.restoreFactoryDefaults();

    /* Set the motors Idle Mode */
    m_IntakeMotor.setIdleMode( CANSparkBase.IdleMode.kCoast );

    /* Set the current limits on the Motors */
    m_IntakeMotor.setSmartCurrentLimit( 35 );

    /* Setup the Motor Encoders */
    m_IntakeEncoder = m_IntakeMotor.getEncoder();

    /* Burn the new settings into the flash. */
    m_IntakeMotor.burnFlash();

    /* Setup Index Beam Break sensor */
    m_IntakeSensor = new DigitalInput( Constants.Intake.k_DIO_IntakeSensorID );
      
  }

  public boolean getIntakeSensor()
  {
    return !m_IntakeSensor.get();
  }

  public boolean isNoteInIntake()
  {
    return m_NoteInIntake;
  }

  public void runIntake( double speed )
  {
    m_IntakeSpeed = speed;
    m_IntakeMotor.set( m_IntakeSpeed );
  }

  public void stopIntake()
  {
    m_IntakeSpeed = 0.0;
    m_IntakeMotor.set( m_IntakeSpeed );
  }


  @Override
  public void periodic() 
  { 
    m_NoteInIntake = m_IntakeSensor.get() ? false : true;

    SmartDashboard.putNumber( "Intake Set Speed:", m_IntakeSpeed );
    SmartDashboard.putNumber( "Intake Actual Speed:", m_IntakeEncoder.getVelocity() );
    SmartDashboard.putNumber( "Intake Motor Current", m_IntakeMotor.getOutputCurrent() );
    SmartDashboard.putBoolean( "Intake Sensor", m_NoteInIntake );
  } 

/***************************** Commands ************************************* */
  public Command runIntakeFastCommand()
  {
    return new RunCommand(()->this.runIntake( Intake.k_IntakeIntakeSpeedFast ), this );
  }

  public Command runIntakeSlowCommand()
  {
    return new RunCommand(()->this.runIntake( Intake.k_IntakeIntakeSpeedSlow ), this );
  }

  public Command stopIntakeCommand()
  {
    return new InstantCommand(()->this.stopIntake(), this );
  }

  public Command ejectIntakeCommand()
  {
    return new RunCommand(()->this.runIntake( Intake.k_IntakeEjectSpeed ), this );
  } 

  /* Triggers */
  //public Trigger IntakeNoteTrigger = new Trigger( () -> this.getIntakeSensor() );

  
}