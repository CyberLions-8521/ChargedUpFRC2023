// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmAndJoint extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax m_jointMotor1 = new CANSparkMax(Constants.MotorControllerIDs.JOINT_MOTOR1, MotorType.kBrushless);
  private final CANSparkMax m_jointMotor2 = new CANSparkMax(Constants.MotorControllerIDs.JOINT_MOTOR2, MotorType.kBrushless);

  //private final SparkMaxLimitSwitch m_limitSwitch = m_jointMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  public final CANSparkMax m_armMotor = new CANSparkMax(Constants.MotorControllerIDs.ARM_MOTOR, MotorType.kBrushless);
  public final AbsoluteEncoder m_jointEncoder = m_jointMotor2.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  public final MotorControllerGroup m_jointGroup = new MotorControllerGroup(m_jointMotor1, m_jointMotor2);

  private final RelativeEncoder m_ArmEncoder = m_armMotor.getEncoder();
  //private final AnalogPotentiometer m_armPotentiometer = new AnalogPotentiometer(0, 933, -30);

  private final PIDController m_PIDArm = new PIDController(0.1, 0, 0);
  private final PIDController m_PIDJoint = new PIDController(0.1, 0, 0);


  public ArmAndJoint() {
    //m_jointEncoder.setPosition(0);
    //m_jointMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    m_jointMotor1.setInverted(true);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public CommandBase retractArm() {
    return runOnce(
      () -> {
        m_armMotor.set(-0.25);
      });
  }

  public CommandBase extendArm() {
    return runOnce(
      () -> {
        m_armMotor.set(0.25);
      });
  }

  public CommandBase PIDArmAndJoint(double SPx, double SPy){
    return run(
      () -> {
        moveToAngleSetpoint(SPx, SPy);
        armExtrusionToSetpoint(SPx, SPy);
      }
    );
  }

  public CommandBase dog(double setpoint){
    return run(
      () -> {
        moveToAngleSetpointRACHEL(setpoint);
      }
    );
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
/* 
  public boolean isLimitSwitchPressed(){
    return m_limitSwitch.isPressed();
  }*/

  public void move(double leftTrigger, double rightTrigger){
    m_jointGroup.set((-leftTrigger + rightTrigger) * 0.1 + 0.02);
    //m_armMotor.set(-leftTrigger * 0.25);
    // if(right) {
    //   m_armMotor.set(0.25);
    // } else if(left) {
    //   m_armMotor.set(-0.25);
    // } else {
    //   m_armMotor.set(0);
    // }
  }

  public double getCurrentAngle(){
    double revolutions = m_jointEncoder.getPosition();
    double angle = 0;
    if (angle < 360){
      angle = (360*revolutions);
    }
    return angle;
  }

  public double getR2Length(){
    final double m = 0;
    final double b = 0;
    return m * m_jointEncoder.getPosition() + b;
  }

  public double getR3Length(){
    return (getR2Length() + Constants.ElevatorConstants.r1);
  }

  public double distanceFormula(double x1, double x2, double y1, double y2){
    return ( Math.sqrt(Math.pow((x2 - x1),2) + Math.pow((y2 -y1),2)));
  }

  public void moveToAngleSetpoint(double SPx, double SPy){
    double output = m_PIDJoint.calculate(getCurrentAngle(), getAngleToSetpoint(SPx, SPy));
    m_jointGroup.set(output);

    SmartDashboard.putNumber("Angle Distance to Setpoint", getAngleToSetpoint(SPx, SPy));
  }

  public void moveToAngleSetpointBigK(double SPx, double SPy){
    double setpoint = getAngleToSetpoint(SPx, SPy);
    double output = m_PIDJoint.calculate(getCurrentAngle(), setpoint);
    m_jointGroup.set(output);
  }

  public void moveToAngleSetpointRACHEL(double wantedAngle){
    double output = m_PIDJoint.calculate(getCurrentAngle(), wantedAngle);
    m_jointGroup.set(output);
  }

  public void armExtrusionToSetpoint(double SPx, double SPy){
    double distanceToSetpoint = distanceFormula(0, SPx, SPy, Constants.ElevatorConstants.height);
    double output = m_PIDArm.calculate(getR3Length(), distanceToSetpoint);
    m_armMotor.set(output);
    SmartDashboard.putNumber("Arm length to Setpoint", distanceToSetpoint);
  }

  public double getAngleToSetpoint(double SPx, double SPy){
    return (Math.toDegrees(Math.atan(SPy/SPx)));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Raw Joint Value", m_jointEncoder.getPosition());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Angle of Joint", getCurrentAngle());
    SmartDashboard.putNumber("R2 Length", getR2Length());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
