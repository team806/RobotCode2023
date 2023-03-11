package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PistonSubsystem;

public class auto extends CommandBase{

    private ArmSubsystem armsys; 
    private PistonSubsystem psub; 
    
    public auto(ArmSubsystem system, PistonSubsystem pSubsystem) {
        armsys = system;
        psub = pSubsystem;
    }

    SequentialCommandGroup commandgroup = new SequentialCommandGroup (

        new RotateArmCommand (armsys,.15).withTimeout(.1),
        new ExtendArmCommand (armsys,.15).withTimeout( .1),
        new OpenClawCommand (psub).withTimeout(.1) 
    );
}



        
