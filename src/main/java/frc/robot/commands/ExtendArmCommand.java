package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendArmCommand extends CommandBase{
    
    private double speed;
    private final ArmSubsystem armSubsystem;

    public ExtendArmCommand(ArmSubsystem armSubsystem, double speed) {
        this.speed = speed;
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setExtensionSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setExtensionSpeed(0.0);
    }
}
