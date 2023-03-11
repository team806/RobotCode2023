package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PistonSubsystem;

public class CloseClawCommand extends CommandBase{

    PistonSubsystem pistonSubsystem;

    public CloseClawCommand(PistonSubsystem pistonSubsystem) {
        this.pistonSubsystem = pistonSubsystem;
        
        addRequirements(pistonSubsystem);
    }  

    @Override    
    public void initialize() {
        pistonSubsystem.close();
    }

    @Override
    public void end(boolean interrupted) {
        pistonSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
