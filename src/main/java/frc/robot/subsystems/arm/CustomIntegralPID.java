// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.arm;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.PIDCommand;

// /** Add your docs here. */
// public class CustomIntegralPID extends PIDController {
//     public CustomIntegralPID(double kp, double ki, double kd) {
//         super(kp, ki, kd);
//     }

//     /**
//      * Since the default PIDController calculate() method does integration badly, use this method instead.
//      * <p>
//      * Simply pass in the custom integral value of your measurement and everything will be great.
//      * @param measurement
//      * @param measurementIntegral
//      * @return
//      */
//     public double calculateWithCustomIntegral(double measurement, double measurementIntegral, double setpoint) {
//         m_measurement = measurement;
//         m_prevError = m_positionError;
//         m_haveMeasurement = true;
    
//         if (m_continuous) {
//           double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
//           m_positionError = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
//         } else {
//           m_positionError = m_setpoint - m_measurement;
//         }
    
//         m_velocityError = (m_positionError - m_prevError) / m_period;
    
//         return m_kp * m_positionError + m_ki * measurementIntegral + m_kd * m_velocityError;
//     }
// }
