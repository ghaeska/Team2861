package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.subsystems.autoDrive_Subsystem;


public class autoArm_Command extends CommandBase
{
    

    public DifferentialDrive m_myRobot;

    private double claw;
    private double shoulder;

    public autoArm_Command(double claw, double shoulder ) 
    {
        this.claw = claw;
        this.shoulder = shoulder;

        
    }
    
    //@Override
    public void inititalize() 
    {

    }

    //@Override

    public void execute()
    {
        Robot.autoArm.setAutoArm( claw, shoulder );
        
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
