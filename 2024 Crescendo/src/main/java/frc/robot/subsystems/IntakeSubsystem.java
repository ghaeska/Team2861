package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.Intake;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class IntakeSubsystem extends SubsystemBase
{

/*------------------------ Private Instance Variables ------------------------*/
  private CANSparkMax m_IntakeMotor;
  private RelativeEncoder m_IntakeEncoder;

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
    SmartDashboard.putNumber( "Intake Set Speed:", m_IntakeSpeed );
    SmartDashboard.putNumber( "Intake Actual Speed:", m_IntakeEncoder.getVelocity() );
    SmartDashboard.putNumber( "Index Motor Current", m_IntakeMotor.getOutputCurrent() );
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
    return new RunCommand(()->this.stopIntake(), this );
  }

  public Command ejectIntakeCommand()
  {
    return new RunCommand(()->this.runIntake( Intake.k_IntakeEjectSpeed ), this );
  }
  
}