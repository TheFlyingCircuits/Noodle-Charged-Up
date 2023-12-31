package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs;

    private PIDController velocityController;
    private ArmFeedforward feedforward;
    private TrapezoidProfile trapezoidProfile;

    private final Timer timer;
    private boolean isMovingToTarget;
    private double setpointRadians;

    private boolean hasBeenHomed = false;

    private Mechanism2d mech;
    private MechanismRoot2d mechRoot;
    private MechanismLigament2d mechArm;

    ShuffleboardTab tab;
    GenericEntry desiredSetpoint;

    public Arm(ArmIO io) {
        System.out.println("[Init] Creating " + this.getClass().getName());
        this.io = io;
        this.inputs = new ArmIOInputsAutoLogged();

        tab = Shuffleboard.getTab("armPos");
        desiredSetpoint = Shuffleboard.getTab("armPos")
        .add("arm position", 0)
        .getEntry();
        
        
        this.velocityController = new PIDController(
                Constants.Arm.KpVoltsPerRadianPerSecond,
                0,
                Constants.Arm.KdVoltsPerRadianPerSecondSquared);

        this.feedforward = new ArmFeedforward(
                Constants.Arm.KsVolts,
                Constants.Arm.KgVolts,
                Constants.Arm.KvVoltsPerRadianPerSecond,
                Constants.Arm.KaVoltsPerRadianPerSecondSquared);

        this.timer = new Timer();


        mech = new Mechanism2d(Constants.Arm.armWidthMeters, Constants.Arm.armLengthMeters);
        mechRoot = mech.getRoot("armRoot", Units.inchesToMeters(5), Units.inchesToMeters(5));
        mechArm = mechRoot.append(new MechanismLigament2d("arm", Constants.Arm.armLengthMeters, 90));
    }

    /**
     * Generates a new trapezoidal profile for the arm to follow.
     * @param setpointRadians - Angular position, in radians of the arm. An angle of 0 represents a completely horizontal, forward-facing arm.
     */
    public void setArmPositionRadians(double setpointRadians) {
        setpointRadians = MathUtil.clamp(setpointRadians, Constants.Arm.minAngleRadians, Constants.Arm.maxAngleRadians);
        this.setpointRadians = setpointRadians;

        trapezoidProfile = new TrapezoidProfile(
            Constants.Arm.motionConstraints,
            new TrapezoidProfile.State(
                setpointRadians, 0.0
            ),
            new TrapezoidProfile.State(
                inputs.armPosition.getRadians(), inputs.armVelocityRadiansPerSecond
            ));

        timer.reset();
        timer.start();

        isMovingToTarget = true;
    }

    private void setArmRadiansPerSecond(double targetRadiansPerSecond) {

        //currently this only incorporates the p gain (velocity rad/s)
        double pidOutputVolts = velocityController.calculate(inputs.armVelocityRadiansPerSecond, targetRadiansPerSecond);
        double feedforwardOutputVolts = feedforward.calculate(inputs.armPosition.getRadians(), targetRadiansPerSecond);


        double positionFeedbackVolts = 0;
        //incorporate position feedback
        if (trapezoidProfile != null) {
            Rotation2d targetPosition = new Rotation2d(trapezoidProfile.calculate(timer.get()).position);
            Rotation2d positionError = targetPosition.minus(inputs.armPosition);
            positionFeedbackVolts = Constants.Arm.KiVoltsPerRadian * positionError.getRadians();
        }

        double totalOutputVolts = pidOutputVolts + feedforwardOutputVolts + positionFeedbackVolts;

        if ((inputs.atBackLimitSwitch && totalOutputVolts > 0) || (inputs.atFrontLimitSwitch && totalOutputVolts < 0)) {
            totalOutputVolts = 0;
        }

        io.setArmVoltage(totalOutputVolts);
    }

    /**
     * positive voltage moves arm forward
     * @param volts
     */
    public void setArmVolts(double volts) {
        io.setArmVoltage(volts);
    }

    /**
     * Calculates desired instantaneous velocity of the arm based off of the trapezoidal motion profile,
     * then calls setArmRadiansPerSecond() to execute this velocity.
     */
    private void followTrapezoidProfile() {

        //holding position if no trapezoid profile is active
        if (!isMovingToTarget) {
            setArmRadiansPerSecond(0);
            return;
        }

        //ends trapezoidal profile if reached desired position
        if (trapezoidProfile.isFinished(timer.get()) && (Math.abs(inputs.armPosition.getRadians() - setpointRadians) <= 0.1)) {
            isMovingToTarget = false;
            return;
        }

        //otherwise, follow trapezoidal profile normally
        double desiredRadiansPerSecond = trapezoidProfile.calculate(timer.get()).velocity;

        setArmRadiansPerSecond(desiredRadiansPerSecond);
    }

    public double getArmPosition() {
        return desiredSetpoint.getDouble(0);
    }

    public boolean atSetpoint() {
        return !isMovingToTarget;
    }

    public boolean atBackLimitSwitch() {
        return inputs.atBackLimitSwitch;
    }

    public double getAverageArmCurrentDraw() {
        return ((inputs.leftPivotMotorArmCurrentAmps + inputs.rightPivotMotorArmCurrentAmps) / 2);
    }

    public void setToBrakeMode(boolean isInBreak) {
        io.setToBrakeMode(isInBreak);
    }

    @Override
    public void periodic() {

        if (inputs.atBackLimitSwitch) {
            io.setArmEncoderPosition(Constants.Arm.maxAngleRadians);
            hasBeenHomed = true;
        }

        io.updateInputs(inputs);

        if (!hasBeenHomed) {
            io.setArmVoltage(1.0);
        }
        else {
            followTrapezoidProfile();
        }


        mechArm.setAngle(inputs.armPosition.getDegrees());

        Logger.getInstance().processInputs("Arm", inputs);
        Logger.getInstance().recordOutput("/arm/mechanism2d", mech);

    }
}
