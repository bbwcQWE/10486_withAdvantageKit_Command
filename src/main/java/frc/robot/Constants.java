// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class ElevatorConstants {
    public static final int ElevatorMotorIdLeft = 11;
    public static final int ElevatorMotorIdRight = 13;

    public static final CurrentLimitsConfigs Amperelimit =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true);
    public static final TalonFXConfiguration configs =
        new TalonFXConfiguration()
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100)
                    .withMotionMagicAcceleration(100))
            .withSlot0(new Slot0Configs().withKP(30).withKI(0).withKD(.4).withKS(0));
  }

  public static class GroundIntakeConstants {
    public static final int GroundIntakeKrakenId = 14;
    public static final int GroundIntakeFalconId = 15;

    public static final int gEncoderId = 2;

    public static final double ENCODER_TO_INTAKE_GEAR_RATIO = 2.0;

    /*地面抓取的PID在这里 */
    public static final double kP = 5;
    public static final double kI = 0;
    public static final double kD = 0.05;

    public static final double kPositionToleranceCycles = 0.005;

    public static final double INTAKE_ANGLE_PREPARE = 0; // 预备位置
    public static final double INTAKE_ANGLE_GROUND = .34; // 地面
    public static final double INTAKE_ANGLE_GRAB = -.2; // L1

    public static final double Maxspeed = .1;

    public static final CurrentLimitsConfigs Amperelimit =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true);
    public static final TalonFXConfiguration configs = new TalonFXConfiguration();
  }
}
