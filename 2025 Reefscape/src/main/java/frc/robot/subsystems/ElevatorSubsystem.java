package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase
{
    // Please note that somethings that were defined last year have changed this year.
    //So instead of CANSparkMax, you need to define it as SparkMax.

    //TODO: TODAY: define two motors, a left and right elevator motor.

    //TODO: TODAY: Define spark pid controller.

    //TODO: TODAY: define a relative encoder for both elevator motors

    //TODO: TODAY: TBD if we need a absolute encoder to measure elevator height.

    //TODO: TODAY: define a Rotation 2d, to we can use it to track height.

  public ElevatorSubsystem()
  {
    //TODO: TODAY: Setup the two motors, you might need to see how this was done 
    // in MAXSwerveModule.java and how it references Configs.java.

  }
  /********************* Helper Functions for Elevator *************************/

  // this function is referenced from setArmSetpoint
  public void setElevatorSetpoint( Rotation2d setpoint_radians ) 
  {        
    
    
  }
  //This function is referenced to getArmAngle()
  // public Rotation2d getElevatorHeight() 
  // {
    
  // }

  // Converts Radians to Degrees
  public double ConvertRadiansToDegrees( double Radians )
  {
    return ( Radians / Math.PI ) * 180;
  }

  //Converts degrees to Radians
  public double degreesToRadians(double degrees) 
  {
    return (degrees * Math.PI) / 180.0;
  }

  //public void runElevator( double setPointRadians )
  {
    
  }

  //public void stopElevator()
  {
    ;
  }
  
  //private boolean onTarget()
  {
    
  }

  //private Rotation2d getError()
  {
    
  }

  /***************************** Commands **************************************/
  /* Commands will go down here.  See lines 186 - 224 in ArmSubystem.java */
}
