package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.Index;
import frc.robot.Constants.Intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeIndexSubsystem extends SubsystemBase
{
  /* Index Stuff */
  private CANSparkMax m_IndexMotor;
  private RelativeEncoder m_IndexEncoder;  
  private DigitalInput m_IndexSensor;
  public boolean m_NoteInIndex;
  private double m_IndexSpeed;

  /* Intake Stuff */
  private CANSparkMax m_IntakeMotor;
  private RelativeEncoder m_IntakeEncoder;
  public DigitalInput m_IntakeSensor;
  public boolean m_NoteInIntake = false;
  private double m_IntakeSpeed;

  public IntakeIndexSubsystem()
  {
    /* Setup the Motor Controllers */
    m_IndexMotor = new CANSparkMax( Index.k_IndexMotorCanId, MotorType.kBrushless );
    m_IntakeMotor = new CANSparkMax( Constants.Intake.k_IntakeMotorCanId, MotorType.kBrushless );

    /* Restore them to defaults */
    m_IndexMotor.restoreFactoryDefaults();
    m_IntakeMotor.restoreFactoryDefaults();

    /* Set the motors Idle Mode */
    m_IndexMotor.setIdleMode( CANSparkBase.IdleMode.kCoast );
    m_IntakeMotor.setIdleMode( CANSparkBase.IdleMode.kCoast );

    /* Set the current limits on the Motors */
    m_IndexMotor.setSmartCurrentLimit( 20 );
    m_IntakeMotor.setSmartCurrentLimit( 35 );

    /* Setup the Motor Encoders */
    m_IndexEncoder = m_IndexMotor.getEncoder();
    m_IntakeEncoder = m_IntakeMotor.getEncoder();

    /* Burn the new settings into the flash. */
    m_IndexMotor.burnFlash();
    m_IntakeMotor.burnFlash();
    
    /* Setup Beam Break sensors */
    m_IndexSensor = new DigitalInput( Constants.Index.k_DIO_IndexSensorID );
    m_IntakeSensor = new DigitalInput( Constants.Intake.k_DIO_IntakeSensorID );
  }

  @Override
  public void periodic() 
  { 
    m_NoteInIndex = m_IndexSensor.get() ? false : true;
    m_NoteInIntake = m_IntakeSensor.get() ? false : true;

    SmartDashboard.putNumber( "Index Set Speed:", m_IndexSpeed );
    SmartDashboard.putNumber( "Index Actual Speed:", m_IndexEncoder.getVelocity() );
    SmartDashboard.putNumber( "Index Motor Current", m_IndexMotor.getOutputCurrent() );
    SmartDashboard.putBoolean( "Index Sensor", m_NoteInIndex );
    
    SmartDashboard.putNumber( "Intake Set Speed:", m_IntakeSpeed );
    SmartDashboard.putNumber( "Intake Actual Speed:", m_IntakeEncoder.getVelocity() );
    SmartDashboard.putNumber( "Intake Motor Current", m_IntakeMotor.getOutputCurrent() );
    SmartDashboard.putBoolean( "Intake Sensor", m_NoteInIntake );
  } 

/*************************** Index Functions **********************************/
  public boolean getIndexSensor()
  {
    return !m_IndexSensor.get();
  }
  
  public boolean isNoteInIndex()
  {
    return m_NoteInIndex;
  }

  public void runIndex( double speed )
  {
    m_IndexSpeed = speed;
    m_IndexMotor.set( m_IndexSpeed );
  }

  public void stopIndex()
  {
    m_IndexSpeed = 0.0;
    m_IndexMotor.set( m_IndexSpeed );
  }

/************************** Intake Functions **********************************/
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



/***************************** Index Commands *********************************/
  public Command runIndexFwdCommand()
  {
    return new RunCommand(()->this.runIndex( Index.k_IndexForwardSpeed ), this );
  }

  public Command runIndexRevCommand()
  {
    return new RunCommand(()->this.runIndex( Index.k_IndexReverseSpeed ), this );
  }

  public Command stopIndexCommand()
  {
    return new RunCommand(()->this.stopIndex(), this );
  }

  public Command feedShooterFromIndexCommand()
  {
    return new RunCommand( () -> this.runIndex( Index.k_IndexForwardSpeed ), this )
    .until( () -> !this.isNoteInIndex() );
  }

/*************************** Intake Commands **********************************/
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

  public Command IntakeToIndexCommand()
  {
    return new RunCommand(()->this.runIntake( Intake.k_IntakeIntakeSpeedSlow ), this )
    .until( () -> this.isNoteInIntake() )
    .andThen( () -> this.runIndex( Index.k_IndexForwardSpeed ), this )
    .until( () -> this.isNoteInIndex() )
    .andThen( () -> this.stopIndex() )
    .andThen( () -> this.stopIntake() );
  }

/*************************** Triggers *****************************************/
  public Trigger IndexNoteSensor = new Trigger( () -> this.getIndexSensor() );
  public Trigger IntakeNoteTrigger = new Trigger( () -> this.getIntakeSensor() );


}
