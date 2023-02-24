// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

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
  private final CANSparkMax m_jointMotor1 = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax m_jointMotor2 = new CANSparkMax(6, MotorType.kBrushless);

  //private final SparkMaxLimitSwitch m_limitSwitch = m_jointMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  public final CANSparkMax m_armMotor = new CANSparkMax(30, MotorType.kBrushless);

  public final MotorControllerGroup m_jointGroup = new MotorControllerGroup(m_jointMotor1, m_jointMotor2);

  //private final RelativeEncoder m_jointEncoder = m_jointMotor.getAlternateEncoder(Type.kQuadrature, 8192);
  //private final AnalogPotentiometer m_armPotentiometer = new AnalogPotentiometer(0, 933, -30);

  private final PIDController m_PID1 = new PIDController(0.1, 0, 0);
  private final PIDController m_PID2 = new PIDController(0.1, 0, 0);


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

/* 
  public CommandBase PIDArmAndJoint(double SPx, double SPy){
    return run(
      () -> {
        moveToAngleSetpoint(SPx, SPy);
        armExtrusionToSetpoint(SPx, SPy);
      }
    );
  }*/

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
    m_jointGroup.set((-leftTrigger + rightTrigger) * 0.1 + 0.009);
  }
/* 
  public double getCurrentAngle(){
    double revolutions = m_jointEncoder.getPosition();
    double angle = (360*revolutions);
    return angle;
  }

  public double getR2Length(){
    return m_armPotentiometer.get();
    //final double m = 0;
    //final double b = 0;
    //(m * m_jointEncoder.getPosition() + b);
  }

  public double getR3Length(){
    return (getR2Length() + Constants.ElevatorConstants.r1);
  }

  public double distanceFormula(double x1, double x2, double y1, double y2){
    return ( Math.sqrt(Math.pow((x2 - x1),2) + Math.pow((y2 -y1),2)));
  }

  public void moveToAngleSetpoint(double SPx, double SPy){
    double output = m_PID1.calculate(getCurrentAngle(), getAngleToSetpoint(SPx, SPy));
    m_jointMotor.set(output);

    SmartDashboard.putNumber("Angle Distance to Setpoint", getAngleToSetpoint(SPx, SPy));
  }

  public void moveToAngleSetpointBigK(double SPx, double SPy){
    double setpoint = getAngleToSetpoint(SPx, SPy);
    double magDif = Math.abs(getCurrentAngle()-setpoint);
    if(magDif < 10 ){
        m_PID1.setPID(0, 0, 0);
    } else if (magDif < 30){
        m_PID1.setPID(0, 0, 0);
    } else if (magDif < 90){
        m_PID1.setPID(0, 0, 0);
    } else {
        m_PID1.setPID(0, 0, 0);
    } 
    double output = m_PID1.calculate(getCurrentAngle(), setpoint);
    m_jointMotor.set(output);
    SmartDashboard.putNumber("Angle Distance to Setpoint", setpoint);
  }

  public void armExtrusionToSetpoint(double SPx, double SPy){
    double distanceToSetpoint = distanceFormula(0, SPx, SPy, Constants.ElevatorConstants.height);
    double output = m_PID2.calculate(getR3Length(), distanceToSetpoint);
    m_armMotor.set(output);
    SmartDashboard.putNumber("Arm length to Setpoint", distanceToSetpoint);
  }

  public double getAngleToSetpoint(double SPx, double SPy){
    return (Math.toDegrees(Math.atan(SPy/SPx)));
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Angle of Joint", getCurrentAngle());
    //SmartDashboard.putNumber("R2 Length", getR2Length());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
