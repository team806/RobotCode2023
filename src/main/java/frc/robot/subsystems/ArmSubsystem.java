package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    
    private static final double TALON_CPR = 2048;
    private static final double EXTENSION_GEAR_RATIO = 32;

    private ArmFeedforward ff;
    
    double feedforward;
    
    private final WPI_TalonFX rotationMotor;
    private final WPI_TalonFX extensionMotor;

    public ArmSubsystem(int rotationID, int extensionID){
        rotationMotor = new WPI_TalonFX(rotationID);

        rotationMotor.configFactoryDefault();

        rotationMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        rotationMotor.setNeutralMode(NeutralMode.Brake);


        extensionMotor = new WPI_TalonFX(extensionID);

        extensionMotor.configFactoryDefault();

        extensionMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        extensionMotor.setNeutralMode(NeutralMode.Brake);

        ff = new ArmFeedforward(0, 1, 0);
    }

    public void setRotationSpeed(double speed) {
        rotationMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopRotationMotor() {
        rotationMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public double getRotationMotorSpeed() {
        return rotationMotor.getMotorOutputPercent();
    }

    public double getArmRotation() {
        return rotationMotor.getSelectedSensorPosition() / TALON_CPR / EXTENSION_GEAR_RATIO;
    }

    public void setArmRotation(double position) {
        rotationMotor.set(ControlMode.Position, position * EXTENSION_GEAR_RATIO * TALON_CPR);
    }

    public double getArmFeedForward() {
        return feedforward;
    }

    public void setExtensionSpeed(double speed) {
        extensionMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopExtensionMotor() {
        extensionMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public double getExtensionMotorSpeed() {
        return extensionMotor.getMotorOutputPercent();
    }

    public double getArmExtension() {
        return extensionMotor.getSelectedSensorPosition() / TALON_CPR / EXTENSION_GEAR_RATIO;
    }

    public void setArmExtension(double position) {
        extensionMotor.set(ControlMode.Position, position * EXTENSION_GEAR_RATIO * TALON_CPR);
    }
    
    @Override
    public void periodic() {
        feedforward = ff.calculate(Units.degreesToRadians(getArmRotation()), Units.degreesToRadians(getRotationMotorSpeed()));
    }

}
