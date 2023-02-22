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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  Joystick m_joystick = new Joystick(0);
  public static final int ENCODER_ID = 3;
  public static final int MOTOR_ID = 1;
  WPI_CANCoder m_encoder;
  CANSparkMax m_motor;
  PIDController m_pid = new PIDController(0, 0, 0);
  double m_setpoint = 0;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_encoder = new WPI_CANCoder(ENCODER_ID);
    m_motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.sensorCoefficient = 360/4096;
    config.unitString = "degrees";
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    config.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.sensorDirection = false;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    m_encoder.configAllSettings(config);
    m_pid.enableContinuousInput(-180, 180);
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
    absoluteTest();
    // relativeTest();
    SmartDashboard.putNumber("relative position", m_encoder.getPosition());
    SmartDashboard.putNumber("Absolute Position", m_encoder.getAbsolutePosition());
    SmartDashboard.putNumber("Difference", m_encoder.getPosition() - m_encoder.getAbsolutePosition());
    SmartDashboard.putNumber("velocity", m_encoder.getVelocity());
    SmartDashboard.putNumber("bus voltage", m_encoder.getBusVoltage());
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
    if (m_joystick.getRawButtonPressed(5)) {
      m_encoder.setPosition(90);
    }
    if (m_joystick.getRawButtonPressed(6)) {
      m_encoder.setPosition(-90);
    }
    if (m_joystick.getRawButtonPressed(7)) {
      m_encoder.setPositionToAbsolute();
    }
    
    m_motor.set(m_pid.calculate(m_encoder.getAbsolutePosition(), m_setpoint));
  }

  public void relativeTest() {
    if (m_joystick.getTriggerPressed()) {
      m_setpoint = 0;
    } 
    if (m_joystick.getTopPressed()) {
      m_setpoint = 360 * 10;
    }
    if (m_joystick.getRawButtonPressed(3)) {
      m_setpoint = 360 * 20;
    }
    if (m_joystick.getRawButtonPressed(4)) {
      m_setpoint = 360 * 30;
    }
    if (m_joystick.getRawButtonPressed(5)) {
      // I would expect it to try to repeat wherever it moved
      m_encoder.setPosition(0);
    }
    m_motor.set(m_pid.calculate(m_encoder.getPosition(), m_setpoint));
  }
}
