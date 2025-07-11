// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  // Initializes an encoder on DIO pins 8 and 9
  // Defaults to 4X decoding and non-inverted
  Encoder elevator_encoder = new Encoder(0, 1);

  private final DigitalInput indexInput = new DigitalInput(7);

  private TalonFX ElevatorMotorLeft = new TalonFX(ElevatorConstants.ElevatorMotorIdLeft);
  private TalonFX ElevatorMotorRight = new TalonFX(ElevatorConstants.ElevatorMotorIdRight);

  public ElevatorSubsystem() {
    // elevator_encoder.setMinRate(10);
    // elevator_encoder.setSamplesToAverage(5);
    elevator_encoder.reset();
    // elevator_encoder.setDistancePerPulse(1.0);

    ElevatorMotorLeft.getConfigurator().apply(ElevatorConstants.configs);
    ElevatorMotorRight.getConfigurator().apply(ElevatorConstants.configs);

    CurrentLimitsConfigs amperageLimit = ElevatorConstants.Amperelimit;
    ElevatorMotorLeft.getConfigurator().refresh(amperageLimit);
    ElevatorMotorRight.getConfigurator().refresh(amperageLimit);
    ElevatorMotorLeft.getConfigurator().apply(amperageLimit);
    ElevatorMotorRight.getConfigurator().apply(amperageLimit);
  }

  public void run(double Elevator) {
    ElevatorMotorLeft.set(Elevator * .5);
    ElevatorMotorRight.set(-Elevator * .5);
  }

  public void stop(double Elevator) {
    ElevatorMotorLeft.set(Elevator * 0);
    ElevatorMotorRight.set(-Elevator * 0);
  }

  public void resetEncoder() {
    elevator_encoder.reset();
    SmartDashboard.putNumber("Ela", 0.0); // 可以在这里也更新一下 SmartDashboard
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Ela", elevator_encoder.getDistance());
    SmartDashboard.putNumber("Ela_Raw", elevator_encoder.get());
    SmartDashboard.putBoolean("Index Pulse", indexInput.get());
  }
}
