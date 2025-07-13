// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Feeder;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */
  private TalonFX FeederTurnerMotor = new TalonFX(FeederConstants.FeederTurnerMotorID);

  private TalonFX FeederWheelMotor = new TalonFX(FeederConstants.FeederWheelMotorId);

  private DutyCycleEncoder Gencoder;
  private PIDController pidController;
  private VoltageOut voltageRequest = new VoltageOut(0);

  private double targetAngleDegrees = FeederConstants.INTAKE_ANGLE_PREPARE;

  private final double physicalZeroOffsetCycles = 0; // 注意改

  public FeederSubsystem() {
    FeederConstants.configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    FeederTurnerMotor.getConfigurator().apply(FeederConstants.configs);
    FeederTurnerMotor.getConfigurator().apply(FeederConstants.Amperelimit);

    Gencoder = new DutyCycleEncoder(FeederConstants.gEncoderId);

    pidController = new PIDController(FeederConstants.kP, FeederConstants.kI, FeederConstants.kD);
    pidController.setTolerance(FeederConstants.kPositionToleranceCycles);
  }

  public void setFeederTargetAngle(double angleDegrees) {
    this.targetAngleDegrees = angleDegrees;
  }

  public double getCurrentFeederAngleDegrees() {
    return Gencoder.get() - physicalZeroOffsetCycles;
  }

  public boolean isAtTargetAngle() {
    return pidController.atSetpoint();
  }

  public void stopMotor() {
    FeederTurnerMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentAngle = getCurrentFeederAngleDegrees();
    double pidOutput = pidController.calculate(currentAngle, targetAngleDegrees);
    double motorVoltage = pidOutput * 12.0 * FeederConstants.Maxspeed;
    SmartDashboard.putNumber("fTarget Angle", targetAngleDegrees);
    SmartDashboard.putNumber("fCurrent Angle (Deg)", currentAngle);
    //// SmartDashboard.putNumber("Intake Angle Error (Deg)", pidController.getPositionError());
    // SmartDashboard.putNumber("Intake PID Output", pidOutput); // 这个值现在是钳制后的
    SmartDashboard.putBoolean("fAt Target", isAtTargetAngle());
    //// SmartDashboard.putNumber("Encoder Raw Value (0-1)", getRawEncoderValue());
    SmartDashboard.putNumber("fMotor Output (V)", motorVoltage);
    SmartDashboard.putNumber("fIntake", Gencoder.get());
    FeederTurnerMotor.setControl(voltageRequest.withOutput(motorVoltage));
  }
}
