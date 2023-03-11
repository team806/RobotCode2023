// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CloseClawCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExtendArmCommand;
import frc.robot.commands.OpenClawCommand;
import frc.robot.commands.RotateArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
//import com.mindsensors.CANLight;
import frc.robot.subsystems.PistonSubsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final ArmSubsystem m_armSubsystem =  new ArmSubsystem(14, 1);
  private final PistonSubsystem m_pistonSubsystem = new PistonSubsystem(0, 1);

  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final CommandXboxController m_codriverController = new CommandXboxController(1);

  //Pneumatics
  //private final Compressor comp = new Compressor(PneumaticsModuleType.CTREPCM);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    m_controller.back().onTrue(new InstantCommand(() -> {
      m_drivetrainSubsystem.zeroGyroscope();
    }));
    
    m_codriverController.rightTrigger().whileTrue(new ExtendArmCommand(m_armSubsystem, 0.4));
    m_codriverController.leftTrigger().whileTrue(new ExtendArmCommand(m_armSubsystem, -0.4));

    m_codriverController.rightBumper().whileFalse(new RotateArmCommand(m_armSubsystem, m_armSubsystem.getArmFeedForward()));

    m_codriverController.rightBumper().whileTrue(new RotateArmCommand(m_armSubsystem, 0.15));
    m_codriverController.leftBumper().whileTrue(new RotateArmCommand(m_armSubsystem, -0.15));

    m_codriverController.y().whileTrue(new CloseClawCommand(m_pistonSubsystem));
    m_codriverController.a().whileTrue(new OpenClawCommand(m_pistonSubsystem));

   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public class Robot {
    
    //CANLight frameLights;
    //DriverStation ds;
    
    public void robotInit() {
        //frameLights = new CANLight(3);
        //ds = DriverStation.getInstance();
        //comp.enableDigital();
    }
//RGB Lights
    public void disabledPeriodic() {
      /*
        if (ds.getAlliance() == DriverStation.Alliance.Red) {
            frameLights.showRGB(255, 0, 0);
        } else if (ds.getAlliance() == DriverStation.Alliance.Blue) {
            frameLights.showRGB(0, 0, 255);
        } else if (ds.getAlliance() == DriverStation.Alliance.Invalid) {
            frameLights.showRGB(255, 200, 0); // yellow
        }
        */
    }

  public void teleopInit() {

    

  }

  public void teleopPeriodic() {

  }
}
}
