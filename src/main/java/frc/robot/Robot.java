// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private enum TestMode {
    RELATIVE,
    ABSOLUTE
  }
  TestMode m_mode = TestMode.RELATIVE;

  Joystick m_joystick = new Joystick(0);
  public static final int CANCODER_ID = 0;
  public static final int LEAD_MOTOR_ID = 1;
  public static final int FOLLOW_MOTOR_ID = 3;
  WPI_CANCoder m_cancoder;
  CANSparkMax m_leadMotor;
  CANSparkMax m_followMotor;
  RelativeEncoder m_internal;
  PIDController m_pid = new PIDController(0.005, 0, 0);
  double m_setpoint = 0;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_cancoder = new WPI_CANCoder(CANCODER_ID);
    m_leadMotor = new CANSparkMax(LEAD_MOTOR_ID, MotorType.kBrushless);
    m_followMotor = new CANSparkMax(FOLLOW_MOTOR_ID, MotorType.kBrushless);
    m_followMotor.follow(m_leadMotor);

    m_internal = m_leadMotor.getEncoder();
    m_internal.setPosition(0);
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.sensorCoefficient = 360/4096;
    config.unitString = "degrees";
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    config.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.sensorDirection = false;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    
    m_cancoder.configAllSettings(config);
    m_cancoder.configMagnetOffset(-m_cancoder.getAbsolutePosition());
    if (m_mode == TestMode.ABSOLUTE) {
      m_pid.enableContinuousInput(-180, 180);
    }
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    if (m_mode == TestMode.ABSOLUTE) {
      // absoluteTest();
    } else {
      // relativeTest();
    }
    m_leadMotor.set(0);
    SmartDashboard.putNumber("Internal Position", m_internal.getPosition());
    SmartDashboard.putNumber("Relative Position", m_cancoder.getPosition());
    SmartDashboard.putNumber("Absolute Position", m_cancoder.getAbsolutePosition());
    SmartDashboard.putNumber("Difference", m_cancoder.getPosition() - m_cancoder.getAbsolutePosition());
    SmartDashboard.putNumber("velocity", m_cancoder.getVelocity());
    SmartDashboard.putNumber("bus voltage", m_cancoder.getBusVoltage());
    SmartDashboard.putNumber("setpoint", m_setpoint);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public void absoluteTest() {
    // Testing absolute
    if (m_joystick.getTriggerPressed()) {
      m_setpoint = 0;
    }
    if (m_joystick.getTopPressed()) {
      m_setpoint = 90;
    }
    if (m_joystick.getRawButtonPressed(3)) {
      m_setpoint = -180;
    }
    if (m_joystick.getRawButtonPressed(4)) {
      m_setpoint = -90;
    }
    if (m_joystick.getRawButtonPressed(11)) {
      m_setpoint = -45; 
    }
    if (m_joystick.getRawButtonPressed(12)) {
      m_setpoint = 45;
    }
    if (m_joystick.getRawButtonPressed(5)) {
      m_cancoder.setPosition(90);
    }
    if (m_joystick.getRawButtonPressed(6)) {
      m_cancoder.setPosition(-90);
    }
    if (m_joystick.getRawButtonPressed(7)) {
      m_cancoder.setPositionToAbsolute();
    }
    m_leadMotor.set(MathUtil.clamp(-m_pid.calculate(m_cancoder.getAbsolutePosition(), m_setpoint), -0.15, 0.15));
  }

  public void relativeTest() {
    if (m_joystick.getTriggerPressed()) {
      m_setpoint = 0;
    } 
    if (m_joystick.getTopPressed()) {
      m_setpoint = 360 * 1;
    }
    if (m_joystick.getRawButtonPressed(3)) {
      m_setpoint = 360 * 2;
    }
    if (m_joystick.getRawButtonPressed(4)) {
      m_setpoint = 360 * 3;
    }
    if (m_joystick.getRawButtonPressed(5)) {
      // I would expect it to try to repeat wherever it moved
      m_cancoder.setPosition(0);
    }
  m_leadMotor.set(MathUtil.clamp(-m_pid.calculate(m_cancoder.getAbsolutePosition(), m_setpoint), -0.15, 0.15));
  }
}
