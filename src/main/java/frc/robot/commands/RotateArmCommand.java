package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.ArmSubsystem;

public class RotateArmCommand extends CommandBase{
    
    private double speed;
    private double effort;
    private final ArmSubsystem armSubsystem;
    private ProfiledPIDController controller;

    public RotateArmCommand(ArmSubsystem armSubsystem, double speed) {
        this.speed = speed;
        this.armSubsystem = armSubsystem;

        controller = new ProfiledPIDController(0.28, 0, 0.028, new Constraints(100, 150));

        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {
        armSubsystem.setRotationSpeed(speed);
    }

    @Override
    public void execute(){
        effort = controller.calculate(armSubsystem.getRotationMotorSpeed(), speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setRotationSpeed(0.0);
    }
}
