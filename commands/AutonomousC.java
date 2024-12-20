
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
//import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class AutonomousC extends Command {
  Intake intake;
  Shooter shooter;
  Timer timer;
  /** Creates a new AutonomousC. */
  public AutonomousC(Intake IT, Shooter ST) {
    intake = IT;
    addRequirements(intake);
    timer = new Timer();
    shooter = ST;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooter.ShootSpeaker(.8);
    timer.start();
    System.out.println(timer.get());
    System.out.println(timer.get());
    if(timer.get() < 1) {
      shooter.ShootSpeaker(.8);
    } if(timer.get() > 1.1 && timer.get() < 1.5) {
      intake.prepareNote(-.4);
    }    if (timer.get()> 1.5 && timer.get() < 1.6) {
      intake.stop();
    } if (timer.get() > 1.5 && timer.get() < 6.5) {
      shooter.stop();
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   intake.stop();
   shooter.stop();
  //  RobotContainer.m_robotDrive.drive(0, 0, .0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // RobotContainer.m_robotDrive.drive(0, 0, .0, true, true);
    return false;
  }
}