// frc.robot.subsystems.elevator.ElevatorIO.java
package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d; // 导入 Rotation2d，用于角度
import org.littletonrobotics.junction.AutoLog; // 导入 AutoLog

public interface ElevatorIO {
    /**
     * 定义电梯的所有输入。
     * AdvantageKit 的 @AutoLog 注解将自动为此类生成 toLog 和 fromLog 方法。
     */
    @AutoLog
    public static class ElevatorIOInputs {
        public double leftMotorPositionRotations = 0.0;
        public double leftMotorVelocityRotationsPerSec = 0.0;
        public double leftMotorAppliedVolts = 0.0;
        public double leftMotorStatorCurrentAmps = 0.0;
        public double leftMotorSupplyCurrentAmps = 0.0;
        public double leftMotorTempCelsius = 0.0;

        public double rightMotorPositionRotations = 0.0;
        public double rightMotorVelocityRotationsPerSec = 0.0;
        public double rightMotorAppliedVolts = 0.0;
        public double rightMotorStatorCurrentAmps = 0.0;
        public double rightMotorSupplyCurrentAmps = 0.0;
        public double rightMotorTempCelsius = 0.0;

    
    }

    /**
     * 更新电梯的输入数据。
     * 这是 IO 实现层从硬件（或模拟器）读取数据并填充 inputs 对象的方法。
     * @param inputs 要更新的输入对象
     */
    public void updateInputs(ElevatorIOInputs inputs);

    /**
     * 设置电梯左电机的motionmagic目标位置。
     * @param targetRotations 目标位置，单位为电机旋转圈数
     */
    public void setMotionMagicPosition(double targetRotations);

    /**
     * 设置电梯左电机的开环输出（例如，电压或百分比）。
     * @param percentOutput 输出百分比 (-1.0 to 1.0)
     */
    public void setLeftMotorOpenLoop(double percentOutput);

    /**
     * 重置电梯左电机的编码器位置。
     * @param positionRotations 要设置的新位置，单位为电机旋转圈数
     */
    public void resetLeftMotorPosition(double positionRotations);
}