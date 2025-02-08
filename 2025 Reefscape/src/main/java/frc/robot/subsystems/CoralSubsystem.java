package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CoralConstants;;
import frc.robot.Configs;


public class CoralSubsystem extends SubsystemBase
{
  /* Define the motors */
  private final SparkFlex m_LeftCoralMotor;
  private final SparkFlex m_RightCoralMotor;
  private final SparkMax m_PivotCoralMotor;

  /* Define spark PID controller */
  private SparkClosedLoopController m_PivotCoralPIDController;

  /* Define Relative motor Encoders */
  private RelativeEncoder m_LeftCoralEncoder;
  private RelativeEncoder m_rightCoralEncoder;

  /* Define Absolute Encoder for pivot */
  private AbsoluteEncoder m_PivotCoralEncoder;

  /* Define Rotation2d for Angular tracking */
  private Rotation2d m_CoralPivotSetpoint = new Rotation2d();

  public CoralSubsystem()
  {
    /* Assign the intake motor, they are spark Flex motors */
    m_LeftCoralMotor = new SparkFlex(Constants.CoralConstants.k_LeftCoralMotorCANId, MotorType.kBrushless );
    m_RightCoralMotor = new SparkFlex(Constants.CoralConstants.k_RightCoralMotorCANId, MotorType.kBrushless );

    /* Assign the Pivot Motor ID */
    m_PivotCoralMotor = new SparkMax(Constants.CoralConstants.k_PivotCoralMotorCANId, MotorType.kBrushless );

    /* Setup the motor Encoders. */
    m_LeftCoralEncoder = m_LeftCoralMotor.getEncoder();
    m_rightCoralEncoder = m_RightCoralMotor.getEncoder();

    /* Configure the left motor */
    m_LeftCoralMotor.configure
    (
      Configs.CoralModule.CoralSparkFlexConfig, 
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

  }
/*********************** Helper Functions for Coral ***************************/
  public void runLeftCoralMotor( double voltage )
  {
    m_LeftCoralMotor.set( voltage );
  }

  public void runPivotCoralMotor( double voltage )
  {
    m_PivotCoralMotor.set( voltage );
  }

  public double getCoralPivotPosition()
  {
    return m_LeftCoralEncoder.getPosition();
  }

  public void resetCoralPivotPosition()
  {
    m_LeftCoralEncoder.setPosition( 0 );
  }

  public void stopCoralPivot()
  {
    m_PivotCoralMotor.set( 0 );
  }

  public void stopLeftCoral()
  {
    m_LeftCoralMotor.set( 0 );
  }

  private void setCoralPivotPosition( double position )
  {
    m_PivotCoralPIDController.setReference( position, ControlType.kPosition );
  }

/****************************** Commands **************************************/





}
