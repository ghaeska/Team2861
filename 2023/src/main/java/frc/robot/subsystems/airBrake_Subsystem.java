package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class airBrake_Subsystem extends SubsystemBase

{
    DoubleSolenoid shoulderSolenoid = null;

    

    public airBrake_Subsystem()
    {
        shoulderSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
        shoulderSolenoid.set(kReverse);
    }

    // @Override
    // public void initDefaultCommand() 
    // {
    //     setDefaultCommand(new Grab_Command());
    // }
    public void AirBrakeOn()
    {
        shoulderSolenoid.toggle();
    }

   
    

}

