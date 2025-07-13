// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  // Initializes an encoder on DIO pins 8 and 9
  // Defaults to 4X decoding and non-inverted
  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  Encoder elevator_encoder = new Encoder(0, 1);
  // DutyCycleEncoder tGencoder = new DutyCycleEncoder(2);

  // private final DigitalInput indexInput = new DigitalInput(7);

  private TalonFX ElevatorMotorLeft = new TalonFX(ElevatorConstants.ElevatorMotorIdLeft);
  private TalonFX ElevatorMotorRight = new TalonFX(ElevatorConstants.ElevatorMotorIdRight);

  public ElevatorSubsystem() {
    // elevator_encoder.setMinRate(10);
    // elevator_encoder.setSamplesToAverage(5);
    elevator_encoder.reset();
    // elevator_encoder.setDistancePerPulse(1.0);
    ElevatorConstants.configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ElevatorMotorLeft.getConfigurator().apply(ElevatorConstants.configs);
    ElevatorMotorRight.getConfigurator().apply(ElevatorConstants.configs);

    CurrentLimitsConfigs amperageLimit = ElevatorConstants.Amperelimit;
    ElevatorMotorLeft.getConfigurator().refresh(amperageLimit);
    ElevatorMotorRight.getConfigurator().refresh(amperageLimit);
    ElevatorMotorLeft.getConfigurator().apply(amperageLimit);
    ElevatorMotorRight.getConfigurator().apply(amperageLimit);

    ElevatorMotorRight.setControl(new Follower(ElevatorConstants.ElevatorMotorIdLeft, true));
  }

  public void run(double Elevator) {
    ElevatorMotorLeft.set(Elevator * .8);
  }

  public void stop(double Elevator) {
    ElevatorMotorLeft.set(Elevator * .08);
  }

  public void resetEncoder() {
    elevator_encoder.reset();
    ElevatorMotorLeft.setPosition(0.0);
    SmartDashboard.putNumber("Ela", 0.0); // 可以在这里也更新一下 SmartDashboard
  }

  public void moveElevator(double targetRotations) {
    ElevatorMotorLeft.setControl(motionMagicRequest.withPosition(targetRotations));
    SmartDashboard.putNumber("Set", targetRotations); // 在 SmartDashboard 上显示你设置的目标
  }

  public double getElevatorPositionRotations() {
    // 直接从 TalonFX 获取位置，它返回的就是转数
    return ElevatorMotorLeft.getPosition().getValueAsDouble();
  }

  public double getElevatorVelocityRotationsPerSec() {
    return ElevatorMotorLeft.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Ela", elevator_encoder.getDistance());
    SmartDashboard.putNumber("Ela_Raw", elevator_encoder.get());
    // --- TalonFX 电梯 Motion Magic 调试参数 ---

    // 1. 电梯当前绝对位置 (以转数 Rotations 为单位)
    SmartDashboard.putNumber("Position", getElevatorPositionRotations());

    // 2. 电梯当前速度 (以转数/秒 Rotations/sec 为单位)
    SmartDashboard.putNumber("Velocity", getElevatorVelocityRotationsPerSec());

    // 3. 电梯左电机施加的电压 (V)
    SmartDashboard.putNumber(
        "leftAppliedVolts", ElevatorMotorLeft.getMotorVoltage().getValueAsDouble());

    // 5. 电梯右电机施加的电压 (V) (作为跟随者，这应该与左电机类似)
    SmartDashboard.putNumber(
        "Elevator_RightMotor_AppliedVolts",
        ElevatorMotorRight.getMotorVoltage().getValueAsDouble());
  }
}
