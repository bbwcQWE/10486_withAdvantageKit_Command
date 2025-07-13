// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GroundIntake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeConstants;

public class GroundIntakeSubsystem extends SubsystemBase {
  /** Creates a new GroundIntakeSubsystem. */
  private TalonFX GIHum = new TalonFX(GroundIntakeConstants.GroundIntakeKrakenId);

  private DutyCycleEncoder Gencoder;
  private PIDController pidController;
  private VoltageOut voltageRequest = new VoltageOut(0);

  private double targetAngleDegrees = GroundIntakeConstants.INTAKE_ANGLE_PREPARE;

  private final double physicalZeroOffsetCycles = 0.54;

  public GroundIntakeSubsystem() {

    GroundIntakeConstants.configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    GIHum.getConfigurator().apply(GroundIntakeConstants.configs);
    GIHum.getConfigurator().apply(GroundIntakeConstants.Amperelimit);

    Gencoder = new DutyCycleEncoder(GroundIntakeConstants.gEncoderId);

    pidController =
        new PIDController(
            GroundIntakeConstants.kP, GroundIntakeConstants.kI, GroundIntakeConstants.kD);
    pidController.setTolerance(GroundIntakeConstants.kPositionToleranceCycles);
  }
  /**
   * 设置抓取装置的目标角度。
   *
   * @param angleDegrees 目标角度，单位为度。
   */
  public void setIntakeTargetAngle(double angleDegrees) {
    this.targetAngleDegrees = angleDegrees;
  }

  /** 获取抓取装置的当前实际角度。 */
  public double getCurrentIntakeAngleDegrees() {
    // 现在直接调用 get() 方法即可，因为它已经根据构造函数配置进行了缩放和偏移。
    return Gencoder.get() - physicalZeroOffsetCycles;
  }

  /**
   * 检查抓取装置是否已达到目标角度。
   *
   * @return 如果达到目标角度，返回 true。
   */
  public boolean isAtTargetAngle() {
    return pidController.atSetpoint();
  }

  public void stopMotor() {
    GIHum.set(0);
  }

  @Override
  public void periodic() {
    double currentAngle = getCurrentIntakeAngleDegrees();
    double pidOutput = pidController.calculate(currentAngle, targetAngleDegrees);
    double motorVoltage = pidOutput * 12.0 * GroundIntakeConstants.Maxspeed;
    // SmartDashboard.putNumber("Intake Target Angle (Deg)", targetAngleDegrees);
    // SmartDashboard.putNumber("Intake Current Angle (Deg)", currentAngle);
    //// SmartDashboard.putNumber("Intake Angle Error (Deg)", pidController.getPositionError());
    // SmartDashboard.putNumber("Intake PID Output", pidOutput); // 这个值现在是钳制后的
    // SmartDashboard.putBoolean("Intake At Target", isAtTargetAngle());
    //// SmartDashboard.putNumber("Encoder Raw Value (0-1)", getRawEncoderValue());
    // SmartDashboard.putNumber("Motor Output (V)", motorVoltage);
    // SmartDashboard.putNumber("Intake", Gencoder.get());
    GIHum.setControl(voltageRequest.withOutput(motorVoltage));
  }
}
