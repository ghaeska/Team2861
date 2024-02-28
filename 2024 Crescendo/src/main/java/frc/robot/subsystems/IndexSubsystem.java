package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.Index;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IndexSubsystem extends SubsystemBase
{
  private CANSparkMax m_IndexMotor;
  private RelativeEncoder m_IndexEncoder;
  
  private DigitalInput m_IndexSensor;

  private double m_IndexSpeed;

  public IndexSubsystem()
  {
    /* Setup the Motor Controllers */
    m_IndexMotor = new CANSparkMax( Index.k_IndexMotorCanId, MotorType.kBrushless );

    /* Restore them to defaults */
    m_IndexMotor.restoreFactoryDefaults();

    /* Set the motors Idle Mode */
    m_IndexMotor.setIdleMode( CANSparkBase.IdleMode.kCoast );

    /* Set the current limits on the Motors */
    m_IndexMotor.setSmartCurrentLimit( 20 );

    /* Setup the Motor Encoders */
    m_IndexEncoder = m_IndexMotor.getEncoder();

    /* Burn the new settings into the flash. */
    m_IndexMotor.burnFlash();
    
    /* Setup Index Beam Break sensor */
    m_IndexSensor = new DigitalInput( Constants.Index.k_DIO_IndexSensorID );
    }

  @Override
  public void periodic() 
  { 
    SmartDashboard.putNumber( "Index Set Speed:", m_IndexSpeed );
    SmartDashboard.putNumber( "Index Actual Speed:", m_IndexEncoder.getVelocity() );
    SmartDashboard.putNumber( "Index Motor Current", m_IndexMotor.getOutputCurrent() );
    SmartDashboard.putBoolean( "Index Sensor", getIndexSensor() );
  } 

  public boolean getIndexSensor()
  {
    return !m_IndexSensor.get();
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


/***************************** Commands ************************************* */
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

  /* Triggers */
  public Trigger IndexNoteSensor = new Trigger( () -> this.getIndexSensor() );


}
