// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder.FeederSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveFeederConmand extends Command {
  private final FeederSubsystem feederSubsystem;
  private final double targetAngle;
  /** Creates a new MoveFeederConmand. */
  public MoveFeederConmand(FeederSubsystem feederSubsystem, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feederSubsystem = feederSubsystem;
    this.targetAngle = targetAngle;
    addRequirements(this.feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feederSubsystem.setFeederTargetAngle(targetAngle);
    ;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return feederSubsystem.isAtTargetAngle();
  }
}
