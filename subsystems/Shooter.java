// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuController.ControlMode;
import frc.sorutil.motor.SuController.IdleMode;
import frc.sorutil.motor.SuSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.assistants.LimelightHelpers;


public class Shooter extends SubsystemBase {
  public SuSparkMax shooterFlyWheelTop;
  public SuSparkMax shooterFlyWheelBottom;
  public SuSparkMax relay;
  public SuSparkMax storage;

  /** Creates a new ExampleSubsystem. */
  public Shooter() {
    MotorConfiguration flywheelControllerConfig = new MotorConfiguration();
    SensorConfiguration flywheelSensorConfig = new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(1));

    MotorConfiguration storageControllerConfig = new MotorConfiguration();
    SensorConfiguration storageSensorConfig = new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(1));

    flywheelControllerConfig.setCurrentLimit(Constants.Shooter.FLYWHEEL_CURRENT_LIMIT);
    flywheelControllerConfig.setMaxOutput(Constants.Shooter.FLYWHEEL_MAX_OUTPUT);
    flywheelControllerConfig.setIdleMode(IdleMode.COAST);

    storageControllerConfig.setCurrentLimit(Constants.Shooter.STORAGE_CURRENT_LIMIT);
    storageControllerConfig.setMaxOutput(Constants.Shooter.STORAGE_MAX_OUTPUT);
    storageControllerConfig.setIdleMode(IdleMode.BRAKE);


    relay = new SuSparkMax(
      new CANSparkMax(Constants.Motor.RELAY, MotorType.kBrushless), "Relay", storageControllerConfig, storageSensorConfig
    );
    storage = new SuSparkMax(
      new CANSparkMax(Constants.Motor.STORAGE, MotorType.kBrushless), "Storage", storageControllerConfig, storageSensorConfig
    );    
    shooterFlyWheelTop = new SuSparkMax(
      new CANSparkMax(Constants.Motor.ROLLER_OUTTAKE_TOP, MotorType.kBrushless), "Top Outtake FlyWheel", flywheelControllerConfig, flywheelSensorConfig
    );
    shooterFlyWheelBottom = new SuSparkMax(
      new CANSparkMax(Constants.Motor.ROLLER_OUTTAKE_BOTTOM, MotorType.kBrushless), "Bottom Outtake FlyWheel", flywheelControllerConfig, flywheelSensorConfig
    );
  }

  public double getDistance() {
    Rotation2d angleToGoal = Rotation2d.fromDegrees(Constants.Shooter.LIMELIGHT_MOUNT_ANGLE)
    .plus(Rotation2d.fromDegrees(LimelightHelpers.getTX("limelight")));
    double distance = (Constants.Shooter.TARGET_HEIGHT - Constants.Shooter.LIMELIGHT_HEIGHT) / angleToGoal.getTan();
    return distance;
  }

  public void retraction() {
    relay.set(ControlMode.PERCENT_OUTPUT, -0.04);
    storage.set(ControlMode.PERCENT_OUTPUT, -0.04);
  }

  public void setShooterFlyWheel(double speakerVoltage){
    shooterFlyWheelTop.set(ControlMode.VOLTAGE, speakerVoltage); 
    shooterFlyWheelBottom.set(ControlMode.VOLTAGE, speakerVoltage);
  }

  public void setFlywheels(String shootType) {
    switch (shootType) {
      case "Speaker":
        setShooterFlyWheel(Constants.Shooter.SPEAKER_VOLTAGE);
        break;
      case "Funnel":
        setShooterFlyWheel(Constants.Shooter.FUNNEL_VOLTAGE);
        break;
      case "General":
        setShooterFlyWheel(Constants.Shooter.GENERAL_VOLTAGE);
        break;
      case "Amp":
        setShooterFlyWheel(Constants.Shooter.AMP_VOLTAGE);
        break;
      default:
        setShooterFlyWheel(Constants.Shooter.DEFAULT_VOLTAGE);
        break;
    }
    relay.set(ControlMode.PERCENT_OUTPUT, 0); // holds note in place
    storage.set(ControlMode.PERCENT_OUTPUT, 0); // holds note in place
  }


  public void outtake() {
    relay.set(ControlMode.PERCENT_OUTPUT, 0.9); 
    storage.set(ControlMode.PERCENT_OUTPUT, 0.9); 
    setShooterFlyWheel(9);
  }

  public void unjamNote() {
    relay.set(ControlMode.PERCENT_OUTPUT, -0.9);   
    storage.set(ControlMode.PERCENT_OUTPUT, -0.9);   
  }

  public void stop(){
    relay.set(ControlMode.PERCENT_OUTPUT, 0);    
    shooterFlyWheelTop.set(ControlMode.VOLTAGE, 0);
    shooterFlyWheelBottom.set(ControlMode.VOLTAGE, 0);  
    storage.set(ControlMode.VOLTAGE, 0);  
  }
  
  public double amperageMotorsOuttake() {
    return Math.max(((CANSparkMax) shooterFlyWheelTop.rawController()).getOutputCurrent(), ((CANSparkMax) shooterFlyWheelBottom.rawController()).getOutputCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double distance = getDistance();
    if (distance <= 60) { //DISTANCE IS IN INCHES
        setShooterFlyWheel(Constants.Shooter.SPEAKER_VOLTAGE);
    } else {
        stop();
    }
    /**  Adding Controller Rumble based on FlyWheel/Shooter readyness  **/ 
    if (shooterFlyWheelBottom.outputVelocity() > 3900) {
      new XboxController(0).setRumble(RumbleType.kBothRumble, 1);
    }
    else {
      new XboxController(0).setRumble(RumbleType.kBothRumble, 0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}