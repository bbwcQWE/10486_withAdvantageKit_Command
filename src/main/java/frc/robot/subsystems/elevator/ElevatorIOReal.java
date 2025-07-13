// frc.robot.subsystems.elevator.ElevatorIOReal.java
package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage; // MotionMagicVoltage 用于设置 MotionMagic
import com.ctre.phoenix6.controls.VoltageOut; // VoltageOut 用于开环电压控制
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX elevatorMotorLeft;
    private final TalonFX elevatorMotorRight;

    // 控制请求对象
    private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(0);
    private final VoltageOut voltageOutRequest = new VoltageOut(0);

    public ElevatorIOReal() {
        elevatorMotorLeft = new TalonFX(ElevatorConstants.ElevatorMotorIdLeft);
        elevatorMotorRight = new TalonFX(ElevatorConstants.ElevatorMotorIdRight);

        // 应用初始配置
        ElevatorConstants.configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorMotorLeft.getConfigurator().apply(ElevatorConstants.configs);
        elevatorMotorRight.getConfigurator().apply(ElevatorConstants.configs);

        // 应用电流限制
        CurrentLimitsConfigs amperageLimit = ElevatorConstants.Amperelimit;
        elevatorMotorLeft.getConfigurator().apply(amperageLimit);
        elevatorMotorRight.getConfigurator().apply(amperageLimit);

        // 设置右电机跟随左电机
        elevatorMotorRight.setControl(new Follower(ElevatorConstants.ElevatorMotorIdLeft, true));

        // 刷新所有信号以获取最新数据
        // 这通常在 periodic() 中完成，但为了确保初始化后能立即获取到值，可以先刷新一次
        elevatorMotorLeft.getPosition().refresh();
        elevatorMotorLeft.getVelocity().refresh();
        elevatorMotorLeft.getMotorVoltage().refresh();
        elevatorMotorLeft.getStatorCurrent().refresh();
        elevatorMotorLeft.getSupplyCurrent().refresh();
        elevatorMotorLeft.getDeviceTemp().refresh();

        elevatorMotorRight.getPosition().refresh();
        elevatorMotorRight.getVelocity().refresh();
        elevatorMotorRight.getMotorVoltage().refresh();
        elevatorMotorRight.getStatorCurrent().refresh();
        elevatorMotorRight.getSupplyCurrent().refresh();
        elevatorMotorRight.getDeviceTemp().refresh();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // 刷新所有信号以确保获取最新数据
        elevatorMotorLeft.getPosition().refresh();
        elevatorMotorLeft.getVelocity().refresh();
        elevatorMotorLeft.getMotorVoltage().refresh();
        elevatorMotorLeft.getStatorCurrent().refresh();
        elevatorMotorLeft.getSupplyCurrent().refresh();
        elevatorMotorLeft.getDeviceTemp().refresh();

        elevatorMotorRight.getPosition().refresh();
        elevatorMotorRight.getVelocity().refresh();
        elevatorMotorRight.getMotorVoltage().refresh();
        elevatorMotorRight.getStatorCurrent().refresh();
        elevatorMotorRight.getSupplyCurrent().refresh();
        elevatorMotorRight.getDeviceTemp().refresh();

        // 填充 inputs 对象
        inputs.leftMotorPositionRotations = elevatorMotorLeft.getPosition().getValueAsDouble();
        inputs.leftMotorVelocityRotationsPerSec = elevatorMotorLeft.getVelocity().getValueAsDouble();
        inputs.leftMotorAppliedVolts = elevatorMotorLeft.getMotorVoltage().getValueAsDouble();
        inputs.leftMotorStatorCurrentAmps = elevatorMotorLeft.getStatorCurrent().getValueAsDouble();
        inputs.leftMotorSupplyCurrentAmps = elevatorMotorLeft.getSupplyCurrent().getValueAsDouble();
        inputs.leftMotorTempCelsius = elevatorMotorLeft.getDeviceTemp().getValueAsDouble();

        inputs.rightMotorPositionRotations = elevatorMotorRight.getPosition().getValueAsDouble();
        inputs.rightMotorVelocityRotationsPerSec = elevatorMotorRight.getVelocity().getValueAsDouble();
        inputs.rightMotorAppliedVolts = elevatorMotorRight.getMotorVoltage().getValueAsDouble();
        inputs.rightMotorStatorCurrentAmps = elevatorMotorRight.getStatorCurrent().getValueAsDouble();
        inputs.rightMotorSupplyCurrentAmps = elevatorMotorRight.getSupplyCurrent().getValueAsDouble();
        inputs.rightMotorTempCelsius = elevatorMotorRight.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setMotionMagicPosition(double targetRotations) {
        // 使用 MotionMagicVoltage 请求设置目标位置
        elevatorMotorLeft.setControl(motionMagicVoltageRequest.withPosition(targetRotations));
    }

    @Override
    public void setLeftMotorOpenLoop(double percentOutput) {
        // 使用 VoltageOut 请求设置开环电压，假设最大输出电压为12V
        elevatorMotorLeft.setControl(voltageOutRequest.withOutput(percentOutput * 12.0));
    }

    @Override
    public void resetLeftMotorPosition(double positionRotations) {
        elevatorMotorLeft.setPosition(positionRotations);
    }
}