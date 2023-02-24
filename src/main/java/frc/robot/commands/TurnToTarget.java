// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TurnToTarget extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Limelight m_limelight;
  private final Drivebase m_drivebase;
  private final String target;
  private int pipelineNum = 0;

  private final PIDController m_pid = new PIDController(0.1, 0, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnToTarget(Limelight limelight, Drivebase drivebase, String selection) {
    m_limelight = limelight;
    m_drivebase = drivebase;
    target = selection;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(target){
      case "april tag":
        pipelineNum = 1;
        break;
      case "cone":
        pipelineNum = 2;
        break;
      case "cube":
        pipelineNum = 3;
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_limelight.setPipeline(pipelineNum);
    double output = m_pid.calculate(m_limelight.getTx(), 0);
    m_drivebase.arcadeDrive(0,output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
