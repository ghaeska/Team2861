package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.SwerveConstants.OIConstants;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.CoralConstants.PivotCoralSetpoints;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoints;
import frc.robot.Constants.CoralConstants;
import frc.robot.Configs;

public class ElevatorSubsystem extends SubsystemBase
{
  public enum ElevatorSetpoint
  {
    k_stow,
    k_feederStation,
    k_l1,
    k_l2,
    k_l3,
    k_l4,
    k_LowA,
    k_HighA;
  }

  //Define the Motors
  private final SparkMax m_LeftEleMotor;
  private final SparkMax m_RightEleMotor;    
  private final SparkMax m_PivotCoralMotor; 

  //Define spark pid controller.
  private SparkClosedLoopController m_LeftElePIDController;
  private SparkClosedLoopController m_PivotCoralPIDController;

  //Define a relative encoder for both elevator motors
  private RelativeEncoder m_LeftEleEncoder;
  private RelativeEncoder m_RightEleEncoder;
  //private RelativeEncoder m_PivotCoralEncoder;

  private AbsoluteEncoder m_AbsoluteEncoder;

  private double m_ElevatorSetpoint = Constants.ElevatorConstants.ElevatorSetpoints.k_stow;
  private double m_PivotCoralSetpoint = Constants.CoralConstants.PivotCoralSetpoints.k_stow;
    
  public ElevatorSubsystem()
  {
    /* Assign the CAN Id's to the motors for the elevator. */
    m_LeftEleMotor  = new SparkMax( Constants.ElevatorConstants.k_LeftElevatorMotorCANId, MotorType.kBrushless );
    m_RightEleMotor = new SparkMax( Constants.ElevatorConstants.k_RightElevatorMotorCANId, MotorType.kBrushless );
    m_PivotCoralMotor = new SparkMax( Constants.CoralConstants.k_PivotCoralMotorCANId, MotorType.kBrushless );

    /* Setup the Elevator Encoder. */
    m_LeftEleEncoder = m_LeftEleMotor.getEncoder();
    m_RightEleEncoder = m_RightEleMotor.getEncoder();
    //m_PivotCoralEncoder = m_PivotCoralMotor.getEncoder();

    m_AbsoluteEncoder = m_PivotCoralMotor.getAbsoluteEncoder();

    /* Setup the Elevator PID Loop. */
    m_LeftElePIDController = m_LeftEleMotor.getClosedLoopController();
    m_PivotCoralPIDController = m_PivotCoralMotor.getClosedLoopController();

    /* Configure the left elevator motor from the configs. */
    m_LeftEleMotor.configure
    (
      Configs.ElevatorModule.ElevatorMotorCfg,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );

    /* Configure the right elevator motor from the configs. */
    m_RightEleMotor.configure
    (
      /* The right motor has to follow the left, set that up. */
      Configs.ElevatorModule.ElevatorMotorCfg.follow
      ( 
        Constants.ElevatorConstants.k_LeftElevatorMotorCANId, 
        false
      ),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );

    /* Configure the Pivot motor. */
    m_PivotCoralMotor.configure
    (
      Configs.CoralModule.CoralSparkMaxConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );

    /* Zero out the encoders at startup. */
    m_LeftEleEncoder.setPosition( 0 );
    m_RightEleEncoder.setPosition( 0 );
    //m_PivotCoralEncoder.setPosition( 0 );


  }
/************************** Smart Dashboard Values ****************************/
@Override
  public void periodic() 
  {
    moveToSetpoint();

    /* Print out the Elevator Encoder positions. */
    SmartDashboard.putNumber( "RightElevatorPosition:", m_RightEleEncoder.getPosition() );
    SmartDashboard.putNumber( "LeftElevatorPosition:", m_LeftEleEncoder.getPosition() );
    //SmartDashboard.putNumber( "CoralPivotPosition:", m_PivotCoralEncoder.getPosition() );

    SmartDashboard.putNumber( "Target Coral Position:", m_PivotCoralSetpoint );
    SmartDashboard.putNumber( "Target Elevator Position:", m_ElevatorSetpoint );

    SmartDashboard.putNumber( "Absolute Encoder Position", m_AbsoluteEncoder.getPosition() );
  }




/********************* Helper Functions for Elevator **************************/
  private void moveToSetpoint()
  {
    m_LeftElePIDController.setReference( m_ElevatorSetpoint, ControlType.kPosition );
    m_PivotCoralPIDController.setReference( m_PivotCoralSetpoint, ControlType.kPosition );
  }    


  

  
  
  public double getElevatorPosition()
  {
    return m_LeftEleEncoder.getPosition();
  }

  public double getElevatorVelocity()
  {
    return m_LeftEleEncoder.getVelocity();
  }

  public void resetElevatorPosition()
  {
    m_LeftEleEncoder.setPosition( 0 );
    m_RightEleEncoder.setPosition( 0 );
    //m_PivotCoralEncoder.setPosition( 0 );
  }
   

  /***************************** Commands **************************************/
  public Command setElevatorSetpointCmd( ElevatorSetpoint setpoint )
  {
    return this.runOnce
    (
      () -> 
      {
        switch( setpoint )
        {
          case k_stow:
            m_ElevatorSetpoint = ElevatorSetpoints.k_stow;
            m_PivotCoralSetpoint = PivotCoralSetpoints.k_stow;
            break;
          case k_feederStation:
            m_ElevatorSetpoint = ElevatorSetpoints.k_feederStation;
            m_PivotCoralSetpoint = PivotCoralSetpoints.k_feederStation;
            break;
          case k_l1:
            m_ElevatorSetpoint = ElevatorSetpoints.k_l1;
            m_PivotCoralSetpoint = PivotCoralSetpoints.k_l1;

            break;
          case k_l2:
            m_ElevatorSetpoint = ElevatorSetpoints.k_l2;
            m_PivotCoralSetpoint = PivotCoralSetpoints.k_l2;

            break;
          case k_l3:
            m_ElevatorSetpoint = ElevatorSetpoints.k_l3;
            m_PivotCoralSetpoint = PivotCoralSetpoints.k_l3;

            break;
          case k_l4:
            m_ElevatorSetpoint = ElevatorSetpoints.k_l4;
            m_PivotCoralSetpoint = PivotCoralSetpoints.k_l4;
            break;
          case k_LowA:
            m_ElevatorSetpoint = ElevatorSetpoints.k_LowA;
            m_PivotCoralSetpoint = PivotCoralSetpoints.k_stow;
        }
      }
    );
  }







  

  

  
}
