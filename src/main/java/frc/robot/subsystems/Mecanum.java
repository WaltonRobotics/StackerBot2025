package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Mecanum extends SubsystemBase {
    private static final class Const {
        public static final int kFlDriveChannel = 0;
        public static final int kFrDriveChannel = 1;
        public static final int kBlDriveChannel = 2;
        public static final int kBrDriveChannel = 3;
        public static final double kDriveSpeedScalar = 0.5;
    }

    private final VictorSP flDrive_ = new VictorSP(Const.kFlDriveChannel);
    private final VictorSP frDrive_ = new VictorSP(Const.kFrDriveChannel);
    private final VictorSP blDrive_ = new VictorSP(Const.kBlDriveChannel);
    private final VictorSP brDrive_ = new VictorSP(Const.kBrDriveChannel);
    private final MecanumDrive mecanumDrive_ = new MecanumDrive(flDrive_, blDrive_, frDrive_, brDrive_);

    public Mecanum() {
        mecanumDrive_.setMaxOutput(Const.kDriveSpeedScalar);
        mecanumDrive_.setDeadband(0.1);
    }

    public Command driveCartesian(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zRotation) {
        return runEnd(() -> {
            mecanumDrive_.driveCartesian(xSpeed.getAsDouble(), ySpeed.getAsDouble(), zRotation.getAsDouble());
        }, () -> {
            mecanumDrive_.stopMotor();
        });
    }
}
