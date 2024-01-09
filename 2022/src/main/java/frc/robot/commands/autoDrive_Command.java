package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.autoDrive_Subsystem;


public class autoDrive_Command extends CommandBase
{
    

    public DifferentialDrive m_myRobot;

    private double speed;

    public autoDrive_Command(double speed) 
    {
        this.speed = speed;

        
    }
    
    //@Override
    public void inititalize() 
    {

    }

    //@Override

    public void execute()
    {
        Robot.autoDrive.setAutoDrive(speed);
        
        //m_myRobot.tankDrive(speed, speed);

    }

    //@Override
    public boolean isFinished()
    {
        return true;

    }
    //@Override
    protected void end(){

    }

    //@Override
    protected void interrupted(){
    }



}
