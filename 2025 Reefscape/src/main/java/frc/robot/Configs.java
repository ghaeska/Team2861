package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.SwerveConstants.ModuleConstants;
import frc.robot.Constants;

public final class Configs 
{
public static final class MAXSwerveModule 
  {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static 
    {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;
      
      /* Driving Motor Configs. */
      drivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
      drivingConfig.encoder
                .positionConversionFactor(drivingFactor) // meters
                .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // These are example gains you may need to them for your own robot!
                .pid(0.04, 0, 0)
                .velocityFF(drivingVelocityFeedForward)
                .outputRange(-1, 1);

      /* Turning Motor Configs. */
      turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
      turningConfig.absoluteEncoder
                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of the steering motor in the MAXSwerve Module.
                .inverted(true)
                .positionConversionFactor(turningFactor) // radians
                .velocityConversionFactor(turningFactor / 60.0); // radians per second
      turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                // These are example gains you may need to them for your own robot!
                .pid(1, 0, 0)
                .outputRange(-1, 1)
                // Enable PID wrap around for the turning motor. This will allow the PID
                // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                // to 10 degrees will go through 0 rather than the other direction which is a
                // longer route.
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);
        }
    }

  public static final class AlgaeModule
  {
    public static final SparkMaxConfig AlgaeMotorConfig = new SparkMaxConfig();

    static
    {
      /* ------------------ Algae Motor Configs. ------------------------- */
      /* Set the idle mode to brake, so the Algae cant fall out. */
      AlgaeMotorConfig.idleMode( IdleMode.kBrake );
      /* set the smart current limit to 30A to prevent motor damage */
      AlgaeMotorConfig.smartCurrentLimit( Constants.AlgaeConstants.k_Algae_MaxCurrent );
    }
  }


  public static final class CoralModule
  {
    public static final SparkMaxConfig CoralSparkMaxConfig = new SparkMaxConfig();
    public static final SparkFlexConfig CoralSparkFlexConfig = new SparkFlexConfig();

    static
    {
      /* ------------------ Coral Motor Configs. ------------------------- */
      CoralSparkMaxConfig.idleMode( IdleMode.kBrake );
      CoralSparkMaxConfig.smartCurrentLimit( Constants.CoralConstants.k_Coral_MaxCurrent );
      CoralSparkMaxConfig.inverted( true );

      CoralSparkFlexConfig.idleMode( IdleMode.kBrake );
      CoralSparkFlexConfig.smartCurrentLimit( Constants.CoralConstants.k_Coral_MaxCurrent );

      CoralSparkMaxConfig
        .closedLoop
        .feedbackSensor( FeedbackSensor.kPrimaryEncoder )
        // Set up the PID values for position control
        .p( 0.1 ) // GTH:TODO: tune.
        .outputRange( -1,1 )
        .maxMotion
        //set MAXMotion parameters for position control.
        .maxVelocity( 1000 ) // GTH:TODO: tune. (rev has 2000)
        .maxAcceleration( 5000 ) // GTH:TODO: tune. (rev has 10000)
        .allowedClosedLoopError( 0.25 ); // GTH:TODO: tune. (rev has .25)

      /* OLD DATA 
      CoralSparkMaxConfig.absoluteEncoder
      .inverted( true )
      .positionConversionFactor( 360 );

      CoralSparkMaxConfig.closedLoop
      .feedbackSensor( FeedbackSensor.kAbsoluteEncoder )
      .outputRange( Constants.CoralConstants.k_PivotMinOutput, Constants.CoralConstants.k_PivotMaxOutput )
      .positionWrappingEnabled( false )
      .positionWrappingMinInput( 0.0 )
      .positionWrappingMaxInput( 360 )
      .pidf
      (
        Constants.CoralConstants.k_PivotCoralMotorP, 
        Constants.CoralConstants.k_PivotCoralMotorI,
        Constants.CoralConstants.k_PivotCoralMotorD,
        Constants.CoralConstants.k_PivotCoralMotorFF
      );
      */
      
    }


  }

  public static final class ElevatorModule
  {
    public static final SparkMaxConfig ElevatorMotorCfg = new SparkMaxConfig();
    
    static 
    {
      /* ------------------ Elevator Motor Configs. ------------------------- */

      /* Set the Idle mode to brake so the motors dont move when powered. */
      ElevatorMotorCfg.idleMode( IdleMode.kBrake );
      /* Set the Smart Current limit to 40A to prevent motor damage. */
      ElevatorMotorCfg.smartCurrentLimit( Constants.ElevatorConstants.k_Ele_MaxCurrent );
      
      ElevatorMotorCfg.inverted( true );
      
      /* Set the PID LOOP up for the Elevator. */
      ElevatorMotorCfg
        .closedLoop
        .feedbackSensor( FeedbackSensor.kPrimaryEncoder )
        // Setup PID values
        .p( 0.1 ) //GTH:TODO: tune.
        .outputRange( -1, 1 )
        //set MAXMotion parameters for position control.
        .maxMotion
        .maxVelocity( 4200 ) // GTH:TODO: tune. (rev has 4200)
        .maxAcceleration( 6000 ) // GTH:TODO: tune. (rev has 6000)
        .allowedClosedLoopError( 0.5 ); // GTH:TODO: tune. (rev has .5)



      /* OLD DATA
      ElevatorMotorCfg.closedLoop
      .feedbackSensor( FeedbackSensor.kPrimaryEncoder )
        .pidf
        ( 
          Constants.ElevatorConstants.k_Ele_PID_P, 
          Constants.ElevatorConstants.k_Ele_PID_I,
          Constants.ElevatorConstants.k_Ele_PID_D,
          Constants.ElevatorConstants.k_Ele_PID_FF 
        )
        .outputRange(-1, 1)
        // .maxMotion
        // .maxVelocity(4200)
        // .maxAcceleration(6000)
        // .allowedClosedLoopError(.5)
        ;

      //ElevatorMotorCfg.encoder
      //.inverted( true );
      */
    }
  }
}
