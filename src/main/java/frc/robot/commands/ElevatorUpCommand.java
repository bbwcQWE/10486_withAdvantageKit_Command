// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorUpCommand extends Command {

  private ElevatorSubsystem elevatorUp;
  /** Creates a new ElevatorUpCommand. */
  public ElevatorUpCommand(ElevatorSubsystem ElevatorUp) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorUp = ElevatorUp;
    addRequirements(this.elevatorUp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // elevatorUp.moveElevator(2000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorUp.run(.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // elevatorUp.moveElevator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
