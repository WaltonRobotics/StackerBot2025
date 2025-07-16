// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Mecanum;
import frc.robot.subsystems.Squeezer;

@Logged
public class Robot extends TimedRobot {

  private final CommandXboxController xbox_ = new CommandXboxController(0);

  private final Mecanum drivetrain_ = new Mecanum();

  @Logged
  private final Elevator elevator_ = new Elevator();
  private final Squeezer squeezer_ = new Squeezer();

  public Robot() {
    // command bindings
    drivetrain_.setDefaultCommand(
      drivetrain_.driveCartesian(
        () -> -xbox_.getLeftY(), xbox_::getLeftX, xbox_::getRightX));

    // TODO: figure out desired elevator controls
    elevator_.setDefaultCommand(
      elevator_.kill()
    );

    xbox_.a().whileTrue(squeezer_.close());
    xbox_.b().whileTrue(squeezer_.open());

    // xbox_.leftBumper().whileTrue(elevator_.goToPosition(0.1));
    xbox_.povUp().whileTrue(elevator_.runDutyCycle(0.5));
    xbox_.povDown().whileTrue(elevator_.runDutyCycle(-0.25));

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
