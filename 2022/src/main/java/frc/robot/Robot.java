// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Claw1_Subsystem;
import frc.robot.subsystems.Claw2_Subsystem;
import frc.robot.subsystems.Claw3_Subsystem;
import frc.robot.subsystems.Lift_Subsystem;
import frc.robot.subsystems.WheelSpin_Subsystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.autoDrive_Subsystem;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;




/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
//Victor
  public static final int

  TEST = 1;
public static Buttons m_buttons;
public static Claw1_Subsystem claw1Speed;
public static Claw2_Subsystem claw2Speed;
public static Claw3_Subsystem claw3Speed;
public static Lift_Subsystem liftSub;
public static WheelSpin_Subsystem wheelSpinSpeed;
public static autoDrive_Subsystem autoDrive;

Timer  myTimer = new Timer();



  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  
  
private static WPI_VictorSPX
  m_leftMotorA = new WPI_VictorSPX(RobotMap.m_leftMotorA),
  m_leftMotorB = new WPI_VictorSPX(RobotMap.m_leftMotorB),
  m_rightMotorA = new WPI_VictorSPX(RobotMap.m_rightMotorA),
  m_rightMotorB = new WPI_VictorSPX(RobotMap.m_rightMotorB);
 /*
  liftMotor = new WPI_VictorSPX(RobotMap.liftMotor),
  spinMotor = new WPI_VictorSPX(RobotMap.spinMotor),
  clawMotor1 = new WPI_VictorSPX(RobotMap.clawMotor1),
  clawMotor2 = new WPI_VictorSPX(RobotMap.clawMotor2),
  clawMotor3 = new WPI_VictorSPX(RobotMap.clawMotor3);
  */


MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotorA, m_leftMotorB);
MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotorA, m_rightMotorB);

CameraServer server;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    // m_leftMotorA.setInverted(true);
    // m_leftMotorB.setInverted(true);

    claw1Speed = new Claw1_Subsystem();
    claw2Speed = new Claw2_Subsystem();
    claw3Speed = new Claw3_Subsystem();
    liftSub = new Lift_Subsystem();
    wheelSpinSpeed = new WheelSpin_Subsystem();

    autoDrive = new autoDrive_Subsystem();

    

    final UsbCamera cornerCamera = CameraServer.startAutomaticCapture();
    //driveCamera.setResolution(640, 480);
    //driveCamera.setFPS(20);
    cornerCamera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 20);

    final UsbCamera driveCamera = CameraServer.startAutomaticCapture();
    driveCamera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 20);

    final UsbCamera climbCamera = CameraServer.startAutomaticCapture();
    climbCamera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 20);

    
    m_buttons = new Buttons();

    m_myRobot = new DifferentialDrive(m_leftMotors, m_rightMotors);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);

    
  }

    //public boolean topLiftLimitValue() {return !topLiftLimitSwitch.get(); } 

  // @Override
  public void autonomousInit()
    {
     
      m_leftMotorA.setInverted(true);
      m_leftMotorB.setInverted(true);
      myTimer.reset();
      myTimer.start();

    }
  

    @Override

    public void autonomousPeriodic() 
    {
      if (myTimer.get() > 0 && myTimer.get() < 7)
      {
        autoDrive.setAutoDrive(.4);//Negative is Forward
      }
      else  
      {
        autoDrive.setAutoDrive(0);
      }
    }

  // public void teleopInit()
  // {
  //   m_leftMotorA.setInverted(true);
  //   m_leftMotorB.setInverted(true);
  // }

  public void teleopPeriodic() 
  {
    CommandScheduler.getInstance().run();

    m_leftMotorA.setInverted(true);
    m_leftMotorB.setInverted(true);
    
    do
    {
      m_myRobot.tankDrive(( .1 * m_leftStick.getY() ), ( .1 *  m_rightStick.getY() ) );
    }
    while( m_rightStick.getRawButton(1) );
    {
      m_myRobot.tankDrive(( .6 * m_leftStick.getY() ), ( .6 * m_rightStick.getY() ) );
    }
    
  }

 
  

  
  
}



