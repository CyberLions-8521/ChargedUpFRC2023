// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Balancing extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivebase m_db;
  private final double limit = 0.5;
  private final PIDController m_pidAngle = new PIDController(0.2, 0.01, 0);
  private final PIDController m_pidStraight = new PIDController(0.4, 0, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Balancing(Drivebase db) {
    m_db = db;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(db);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_db.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double outputAngle = m_pidAngle.calculate(m_db.m_gyro.getRoll(), 0);
    double outputStraight = -m_pidStraight.calculate(m_db.getHeading(), 0);
    m_db.arcadeDriveWithoutLimit((Math.abs(outputAngle) > limit) ?  limit * Math.signum(outputAngle) : outputAngle, 0);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return m_pidAngle.atSetpoint();
  }
}
