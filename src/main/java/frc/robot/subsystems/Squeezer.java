package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Squeezer extends SubsystemBase {
    private static final class Const {
        public static final int kMotorCANID = 1;

        public static final double kCloseDuty = 0.75;
        public static final double kOpenDuty = -0.25;

        public static final TalonSRXConfiguration kMotorConfig = new TalonSRXConfiguration();
        static {
            kMotorConfig.peakCurrentDuration = 1000; // milliseconds
            kMotorConfig.peakCurrentLimit = 10;
            kMotorConfig.continuousCurrentLimit = 5; // amps
        }
    }

    private final TalonSRX motor_ = new TalonSRX(Const.kMotorCANID);

    public Squeezer() {
        motor_.configFactoryDefault();
        motor_.configAllSettings(Const.kMotorConfig);
        motor_.setNeutralMode(NeutralMode.Brake);
    }

    private Command move(double dutycycle) {
        return runEnd(
            () -> motor_.set(TalonSRXControlMode.PercentOutput, dutycycle),
            () -> motor_.set(TalonSRXControlMode.PercentOutput, 0)
        );
    }

    public Command open() { return move(Const.kCloseDuty); }
    public Command close() { return move(Const.kOpenDuty); }

    public Command kill() {
        return runOnce(() -> {
            motor_.set(TalonSRXControlMode.PercentOutput, 0);
        });
    }
}
