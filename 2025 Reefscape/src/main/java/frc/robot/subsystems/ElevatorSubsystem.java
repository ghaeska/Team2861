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

  public ElevatorSubsystem()
  {
    /* Assign the CAN Id's to the motors for the elevator. */
    //TODO TODAY: use the defines from constants.java to have the motor ID's here.
    m_LeftEleMotor  = new SparkMax( Constants.ElevatorConstants./*leftmotorplaceholder*/, MotorType.kBrushless );
    m_RightEleMotor = new SparkMax( Constants.ElevatorConstants./*rightmotorplaceholder*/, MotorType.kBrushless );

    /* Setup the Elevator Encoder. */
    m_LeftEleEncoder = m_LeftEleMotor.getEncoder();

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
        //TODO TODAY: change this to the right motor ID
        Constants.ElevatorConstants./*rightmotorplaceholder*/, 
        true
      ),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters 
    );
  }  
  /********************* Helper Functions for Elevator *************************/
    
  public void runElevator( double voltage )
  {
    m_LeftEleMotor.set( voltage );
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

  /*TODO TODAY:  All commands below need to be created.
   all command run in the following format:

    return new RunCommand(()->this.*FunctionFromAbove*( *DataThatFunctionNeeds* ), this );

    Note: *FunctionFromAbove* is probably going to mainly be setElePosition()
          and *DataThatFunctionNeeds* is a position height.  That is something you
          are going to have defined in the constants.java file.  Something like:
          ElevatorConstants.k_Ele_SrcHeight
  */
  /* Lift to the coral Source Command. */
  public Command ElevatorToSourceCmd()
  {

  }
  
  /* Lift to L1 reef Command. */
  public Command ElevatorToL1Cmd()
  {

  }

  /* Lift to L2 reef Command. */
  public Command ElevatorToL2Cmd()
  {

  }

  /* Lift to L3 reef Command. */
  public Command ElevatorToL3Cmd()
  {

  }

  /* Lift to L4 reef Command. */
  public Command ElevatorToL4Cmd()
  {

  }

  /* Lift to top Algea in reef Command. */
  public Command ElevatorToTopAlgaeCmd()
  {

  }

  /* Lift to Proccessor Command. */
  public Command ElevatorToProcessorCmd()
  {
    
  }



  /* Manual Lifting of Elevator Command. */
  public Command ElevatorManualCmd(CommandXboxController controller )
  {
    //GTH TODO: update this so that the controller can move the elevator faster.
    return new RunCommand(()->this.runElevator( -controller.getLeftY() * .1 ), this );
  }
}
