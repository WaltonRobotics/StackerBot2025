package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Elevator extends SubsystemBase {
    private static final class Const {
        public static final int kMotorCANID = 1;
        public static final double kP = 5;

        public static final double kForwardLimit = 0.75;
        public static final double kReverseLimit = 0.25;

        public static final TalonSRXConfiguration kMotorConfig = new TalonSRXConfiguration();
        static {
            kMotorConfig.peakCurrentDuration = 1000; // milliseconds
            kMotorConfig.peakCurrentLimit = 25;
            kMotorConfig.continuousCurrentLimit = 10; // amps

            kMotorConfig.forwardSoftLimitEnable = false;
            kMotorConfig.forwardSoftLimitThreshold = kForwardLimit;

            kMotorConfig.reverseSoftLimitEnable = false;
            kMotorConfig.reverseSoftLimitThreshold = kReverseLimit;

            kMotorConfig.slot0.kP = kP;
            kMotorConfig.slot0.kI = 0;
            kMotorConfig.slot0.kD = 0;
            kMotorConfig.slot0.kF = 0;
            // kMotorConfig.motionCruiseVelocity = 100;
            // kMotorConfig.motionAcceleration = 100;

            kMotorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.Analog;
            kMotorConfig.primaryPID.selectedFeedbackCoefficient = 1;
        }
    }

    private final TalonSRX motor_ = new TalonSRX(Const.kMotorCANID);

    public Elevator() {
        motor_.configFactoryDefault();
        motor_.configAllSettings(Const.kMotorConfig);
        motor_.setInverted(true);
        motor_.setNeutralMode(NeutralMode.Brake);
    }

    private double positionClamp(double requested) {
        return MathUtil.clamp(requested, Const.kReverseLimit, Const.kForwardLimit);
    }

    public Command runDutyCycle(double dutycycle) {
        return runEnd(
            () -> { motor_.set(TalonSRXControlMode.PercentOutput, dutycycle); },
            () -> { motor_.set(TalonSRXControlMode.PercentOutput, 0.0); }
        );
    }

    public Command goToPosition(double position) {
        return runOnce(() -> {
            motor_.set(TalonSRXControlMode.Position, positionClamp(position));
        });
    }

    public Command goToPosition(DoubleSupplier positionSup) {
        return run(() -> {
            motor_.set(TalonSRXControlMode.Position, positionClamp(positionSup.getAsDouble()));
        });
    }

    public Command kill() {
        return runOnce(() -> {
            motor_.set(TalonSRXControlMode.PercentOutput, 0);
        });
    }

    @Logged(name = "PotPosition")
    public double getSensorPosition() {
        return motor_.getSelectedSensorPosition();
    }
}
