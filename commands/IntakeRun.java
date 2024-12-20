// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;

public class IntakeRun extends Command {
  Intake intake;
  DriveSubsystem driveSubsystem;
  
  
  /** Creates a new IntakeBall. */
  public IntakeRun(Intake i, DriveSubsystem DS) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSubsystem = DS;
    intake = i;
    // addRequirements(driveSubsystem);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (driveSubsystem.getDistance() < .3) {
        intake.stop();
    } else {
        intake.intakeRun(-.3);
        intake.prepareNote(-.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
