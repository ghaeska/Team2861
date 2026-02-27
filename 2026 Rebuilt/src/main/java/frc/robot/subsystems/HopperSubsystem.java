package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IntakeConstants.IntakeArmSetpoints;
import frc.robot.Configs;

public class HopperSubsystem extends SubsystemBase
{
  /* Define the Motors */
  private final SparkMax m_HopperMotor;
  
  /* Define the relative encoders for the Hopper. */
  private RelativeEncoder m_HopperEncoder;


  public HopperSubsystem()
  {
    /* Setup the motors */
    m_HopperMotor = new SparkMax( Constants.HopperConstants.k_HopperMotorCANId, MotorType.kBrushless );

    /* Setup the Encoder */
    m_HopperEncoder = m_HopperMotor.getEncoder();

    /* Configure the Motor */
    m_HopperMotor.configure
    (
      Configs.HopperModule.HopperMotorConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );


  }

  /************************** Smart Dashboard Values ****************************/
@Override
  public void periodic() 
  {
    /* Print out the Hopper important data. */
    /* Hopper Motor Data. */
    SmartDashboard.putNumber( "Hopper | Position:", m_HopperEncoder.getPosition() );
    SmartDashboard.putNumber( "Hopper | Velocity:", m_HopperEncoder.getVelocity() );
    SmartDashboard.putNumber( "Hopper | Applied Output", m_HopperMotor.getAppliedOutput());
    SmartDashboard.putNumber( "Hopper | Current", m_HopperMotor.getOutputCurrent());
  }

  /********************* Helper Functions for Intake *************************/
  public double getHopperPosition()
  {
    /* Just do left arm values as the right arm follows it. */
    return m_HopperEncoder.getPosition();
  }

  public double getHopperVelocity()
  {
    /* Just do left arm values as the right arm follows it. */
    return m_HopperEncoder.getVelocity();
  }

  public void resetHopperPosition()
  {
    /* Just do left arm values as the right arm follows it. */
    //m_HopperEncoder.setPosition( IntakeArmSetpoints.k_Stow );
    //m_HopperEncoder.setPosition( IntakeArmSetpoints.k_Stow );
  }

  public void setHopperVoltage( double voltage )
  {
    m_HopperMotor.setVoltage( voltage );
  }

/***************************** Commands **************************************/
  public Command runHopperCommand()
  {
    return Commands.startEnd
    ( 
      () -> m_HopperMotor.setVoltage( 4 ), 
      () -> m_HopperMotor.setVoltage( 0 )
    );
  }

  public Command revHopperCommand()
  {
    return Commands.startEnd
    ( 
      () -> m_HopperMotor.setVoltage( -8 ), 
      () -> m_HopperMotor.setVoltage( 0 )    
    );
  }

  public Command runHopperVoltageCommand( double voltage)
  {
    return Commands.runEnd
    ( 
      () -> m_HopperMotor.setVoltage( voltage ), 
      () -> m_HopperMotor.setVoltage( 0 )
    );
  }


}
