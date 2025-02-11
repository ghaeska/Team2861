package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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
import frc.robot.Configs;

public class ElevatorSubsystem extends SubsystemBase
{
    //Define the Motors
    private final SparkMax m_LeftEleMotor;
    private final SparkMax m_RightEleMotor;    

    //Define spark pid controller.
    private SparkClosedLoopController m_LeftElePIDController;

    //Define a relative encoder for both elevator motors
    private RelativeEncoder m_LeftEleEncoder;
    private RelativeEncoder m_RightEleEncoder;

  public ElevatorSubsystem()
  {
    /* Assign the CAN Id's to the motors for the elevator. */
    m_LeftEleMotor  = new SparkMax( Constants.ElevatorConstants.k_LeftElevatorMotorCANId, MotorType.kBrushless );
    m_RightEleMotor = new SparkMax( Constants.ElevatorConstants.k_RightElevatorMotorCANId, MotorType.kBrushless );

    /* Setup the Elevator Encoder. */
    m_LeftEleEncoder = m_LeftEleMotor.getEncoder();
    m_RightEleEncoder = m_RightEleMotor.getEncoder();

    /* Setup the Elevator PID Loop. */
    m_LeftElePIDController = m_LeftEleMotor.getClosedLoopController();

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
  }
/************************** Smart Dashboard Values ****************************/
@Override
  public void periodic() 
  {
    /* Print out the Elevator Encoder positions. */
    SmartDashboard.putNumber("RightElevatorPosition:", m_RightEleEncoder.getPosition() );
    SmartDashboard.putNumber("LeftElevatorPosition:", m_LeftEleEncoder.getPosition() );
  }




/********************* Helper Functions for Elevator **************************/
    
  public void runElevator( double voltage )
  {
    if( m_LeftEleEncoder.getPosition() > Constants.ElevatorConstants.k_Ele_MaxHeight )
    {
      if( voltage > 0 )
      {
        m_LeftEleMotor.set( 0 );
      }
      else
      {
        m_LeftEleMotor.set( voltage );
      }
      
    }
    else if( m_LeftEleEncoder.getPosition() < 0 )
    {
      if( voltage < 0 )
      {
        m_LeftEleMotor.set( 0 );
      }
      else
      {
        m_LeftEleMotor.set( voltage );
      }

    }
    else
    {
       m_LeftEleMotor.set( voltage );
    }
   
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
  }

  public void stopElevator()
  {
    m_LeftEleMotor.setVoltage( 0 );
  }

  private void setElePosition( double position )
  {
    m_LeftElePIDController.setReference(position, ControlType.kPosition );
    //m_LeftEleMotor.getClosedLoopController().setReference(position, ControlType.kPosition );
  }

  /***************************** Commands **************************************/
  
  /* Lift to the coral Source Command. */
  public Command ElevatorToSourceCmd()
  {
    return new RunCommand(()->this.setElePosition( ElevatorConstants.k_Ele_SrcHeight ), this );
  }
  /* Lift to L1 reef Command. */
  public Command ElevatorToL1Cmd()
  {
    return new RunCommand(()->this.setElePosition( ElevatorConstants.k_Ele_L1Height ), this );
  }
  /* Lift to L2 reef Command. */
  public Command ElevatorToL2Cmd()
  {
    return new RunCommand(()->this.setElePosition( ElevatorConstants.k_Ele_L2Height ), this );
  }
  /* Lift to L3 reef Command. */
  public Command ElevatorToL3Cmd()
  {
    return new RunCommand(()->this.setElePosition( ElevatorConstants.k_Ele_L3Height ), this );
  }
  /* Lift to L4 reef Command. */
  public Command ElevatorToL4Cmd()
  {
    return new RunCommand(()->this.setElePosition( ElevatorConstants.k_Ele_L4Height ), this );
  }
  /* Lift to top Algea in reef Command. */
  public Command ElevatorToTopAlgaeCmd()
  {
    return new RunCommand(()->this.setElePosition( ElevatorConstants.k_Ele_HighAlgaeHeight ), this );
  }
  /* Lift to Proccessor Command. */
  public Command ElevatorToProcessorCmd()
  {
    return new RunCommand(()->this.setElePosition( ElevatorConstants.k_Ele_ScoreAlgaeHeight ), this );
  }

  public Command ElevatorManualUp( double speed )
  {
    return new RunCommand( ()->this.runElevator( speed ), this );
  }

  public Command ElevatorManualDown( double speed )
  {
    return new RunCommand( ()->this.runElevator( -speed ), this );
  }

  /* Manual Lifting of Elevator Command. */
  public Command ElevatorManualCmd(CommandXboxController controller )
  {
    return new RunCommand(()->this.runElevator( -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband) * 1.0 ), this );
  }
}
