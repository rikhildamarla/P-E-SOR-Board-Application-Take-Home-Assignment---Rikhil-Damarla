// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import frc.sorutil.motor.SuController.ControlMode;

import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SpeakerShoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter Shooter;
  private final LinearFilter lf;
  
  public SpeakerShoot(Shooter subsystem) {
    Shooter = subsystem;
    this.lf = LinearFilter.movingAverage(10);
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("UTBAE", Shooter.shooterFlyWheelBottom.outputVelocity());
    double value = Shooter.shooterFlyWheelBottom.outputVelocity();
    Shooter.setFlywheels("Speaker");
    if (value >= Constants.Shooter.FLYWHEEL_SPEAKER_SHOOT_RPM) { // TODO: Change Values to Constants
        Shooter.outtake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
