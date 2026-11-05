
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import frc.robot.Constants.ElevatorConstants;

/*
 * Guidelines for setting up the elevator subsystem:
 * 1. Identify the leader motor on the elevator by making it blink.
 * 2. Determine the rotation direction by manually operating the elevator.
 * 3. Set the inversion based on the required direction for upward movement.
 *
 * 4. Proceed with additional calibration as needed.
 */

public class Elevator extends SubsystemBase {

    // Creates the two elevator motors and the encoder
    private final TalonFX primaryMotor = new TalonFX(ElevatorConstants.kElevatorLeaderID);
    private final TalonFX secondaryMotor = new TalonFX(ElevatorConstants.kElevatorFollowerID);
    private final CANcoder absoluteEncoder = new CANcoder(ElevatorConstants.kCANCoderID);

    // Limit switch tells us when the elevator reaches the bottom
    private final DigitalInput bottomSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchPort);
    private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);

    public Elevator() {
        // Sets up the motor settings
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        // Sets the motor direction and coast mode
        MotorOutputConfigs outputConfig = new MotorOutputConfigs();
        outputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        outputConfig.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput = outputConfig;

        // Uses the encoder to track the elevator position
        FeedbackConfigs feedbackConfig = new FeedbackConfigs();
        feedbackConfig.RotorToSensorRatio = 1;
        feedbackConfig.SensorToMechanismRatio = 1;
        feedbackConfig.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        feedbackConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorConfig.Feedback = feedbackConfig;

        // Limits the motor current to 40 amps
        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimit = Amps.of(40).in(Amps);
        currentConfig.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits = currentConfig;

        // Controls how fast the elevator moves and accelerates
        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = RotationsPerSecond.of(12).in(RotationsPerSecond);
        motionMagicConfig.MotionMagicAcceleration = RotationsPerSecondPerSecond.of(10).in(RotationsPerSecondPerSecond);
        motionMagicConfig.MotionMagicJerk = RotationsPerSecondPerSecond.per(Second).of(0).in(RotationsPerSecondPerSecond.per(Second));
        motorConfig.MotionMagic = motionMagicConfig;

        // These values help the motor reach and hold its position
        Slot0Configs slotConfig = new Slot0Configs();
        slotConfig.kP = 5.0;
        slotConfig.kI = 2.0;
        slotConfig.kD = 0.0;
        slotConfig.kG = 0.2;
        slotConfig.kS = 0.0;
        slotConfig.kV = 0.0;
        slotConfig.kA = 0.0;
        motorConfig.Slot0 = slotConfig;

        // Applies the settings to both motors
        primaryMotor.getConfigurator().apply(motorConfig);
        secondaryMotor.getConfigurator().apply(motorConfig);

        // Makes the second motor follow the first motor
        secondaryMotor.setControl(new Follower(ElevatorConstants.kElevatorLeaderID, true));

        // Resets both motor positions to zero
        primaryMotor.getConfigurator().setPosition(0);
        secondaryMotor.getConfigurator().setPosition(0);
    }

    // Checks if the bottom limit switch is pressed
    public boolean getLimitSwitch() {
        return !bottomSwitch.get();
    }

    // Moves the elevator to a specific rotation position
    public void moveMotor(double rotations) {
        primaryMotor.setControl(motionRequest.withPosition(rotations));
    }

    // Creates a command to move the elevator
    public Command moveMotorCommand(double rotations) {
        return run(() -> moveMotor(rotations));
    }

    // Sets the motor output to zero
    public Command goLimpCommand() {
        return run(() -> primaryMotor.setControl(new DutyCycleOut(0)));
    }

    // Moves the elevator to level 1
    public Command level1() {
        return moveMotorCommand(0.2);
    }

    // Moves the elevator to level 2
    public Command level2() {
        return moveMotorCommand(0.6);
    }

    // Moves the elevator to level 3
    public Command level3() {
        return moveMotorCommand(1.2);
    }

    // Moves the elevator close to its starting position
    public Command approachZero() {
        return moveMotorCommand(0.01);
    }

    // Gets the current position of the main motor
    public double getLeaderEncoderPosition() {
        return primaryMotor.getPosition().getValueAsDouble();
    }

    // Checks if the elevator is close to zero
    public boolean closeToZero() {
        return getLeaderEncoderPosition() < 0.02;
    }

    // Moves the elevator near zero and then stops the motor output
    public Command goToZero() {
        return Commands.sequence(
            moveMotorCommand(0.05).withTimeout(4),
            goLimpCommand()
        );
    }

    @Override
    public void periodic() {
        // Sends motor positions and limit switch status to SmartDashboard
        SmartDashboard.putNumber("Primary Motor Rotations", primaryMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Secondary Motor Rotations", secondaryMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Limit Switch", getLimitSwitch());
    }
}
