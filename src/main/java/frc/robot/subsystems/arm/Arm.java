package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    public enum ArmPosition {
        Intake,
        Low,
        Mid,
        High,
        Default
    }

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs;

    private PIDController velocityController;
    private ArmFeedforward feedforward;
    private TrapezoidProfile trapezoidProfile;

    private final Timer timer;
    private double lastTime;

    private boolean isMovingToTarget;
    private double setpointDegrees;

    private TrapezoidProfile.Constraints motionConstraints;

    private Mechanism2d mech;
    private MechanismRoot2d mechRoot;
    private MechanismLigament2d mechArm;

    public Arm(ArmIO io) {
        this.io = io;
        this.inputs = new ArmIOInputsAutoLogged();

        this.velocityController = new PIDController(
                Constants.Arm.Kp,
                Constants.Arm.Ki,
                Constants.Arm.Kd);

        this.feedforward = new ArmFeedforward(
                Constants.Arm.Ks,
                Constants.Arm.Kg,
                Constants.Arm.Kv,
                Constants.Arm.Ka);

        this.timer = new Timer();
        this.lastTime = 0.0;

        this.motionConstraints = new TrapezoidProfile.Constraints(
            Constants.Arm.maxDesiredVelocityDegreesPerSecond,
            Constants.Arm.maxDesiredAccelerationDegreesPerSecond);


        mech = new Mechanism2d(Constants.Arm.armWidthMeters, Constants.Arm.armLengthMeters);
        mechRoot = mech.getRoot("armRoot", Units.inchesToMeters(-7.581445), 0);
        mechArm = mechRoot.append(new MechanismLigament2d("arm", Constants.Arm.armLengthMeters, 90));
    }

    /**
     * Generates a new trapezoidal profile for the arm to follow.
     * @param setpointDegrees - Angular position, in degrees of the arm. An angle of 0 represents a completely horizontal, forward-facing arm.
     */
    public void setArmPositionDegrees(double setpointDegrees) {
        setpointDegrees = MathUtil.clamp(setpointDegrees, Constants.Arm.minAngleDegrees, Constants.Arm.maxAngleDegrees);
        this.setpointDegrees = setpointDegrees;

        trapezoidProfile = new TrapezoidProfile(
            motionConstraints,
            new TrapezoidProfile.State(
                setpointDegrees, 0.0
            ),
            new TrapezoidProfile.State(
                inputs.armPosition.getDegrees(), inputs.armRotateDegreesPerSecond
            ));

        timer.reset();
        timer.start();

        isMovingToTarget = true;
        mechArm.setAngle(setpointDegrees);
    }

    public void setArmPosition(ArmPosition position) {
        switch (position) {
            case Intake:
                setpointDegrees = Constants.Arm.intakePositionRadians;
                break;
            case Low:
                setpointDegrees = Constants.Arm.lowPositionRadians;
                break;
            case Mid:
                setpointDegrees = Constants.Arm.midPositionRadians;
                break;
            case High:
                setpointDegrees = Constants.Arm.highPositionRadians;
                break;
            case Default:
            default:
                setpointDegrees = Constants.Arm.defaultPositionRadians;
                break;
        }
        setArmPositionDegrees(setpointDegrees);
        mechArm.setAngle(setpointDegrees);
    }

    private void setArmDegreesPerSecond(double targetDegreesPerSecond) {

        //TODO: INCORPORATE CUSTOM POSITION FEEDBACK RATHER THAN USING PIDCONTROLLER INTEGRAL IN ORDER TO AVOID INTEGRAL WINDUP
        double pidOutputVolts = velocityController.calculate(inputs.armRotateDegreesPerSecond, targetDegreesPerSecond);
        double feedforwardOutputVolts = feedforward.calculate(inputs.armPosition.getRadians(), targetDegreesPerSecond);

        double totalOutputVolts = pidOutputVolts + feedforwardOutputVolts;

        if ((inputs.atBackLimitSwitch && totalOutputVolts < 0) || (inputs.atFrontLimitSwitch && totalOutputVolts > 0)) {
            totalOutputVolts = 0;
        }

        io.setArmVoltage(totalOutputVolts);
    }

    /**
     * Calculates desired instantaneous velocity of the arm based off of the trapezoidal motion profile,
     * then calls setArmDegreesPerSecond to execute this velocity.
     */
    private void followTrapezoidProfile() {

        //holding position if no trapezoid profile is active
        if (!isMovingToTarget) {
            setArmDegreesPerSecond(0);
            return;
        }

        //ends trapezoidal profile if reached desired position
        if (trapezoidProfile.isFinished(timer.get()) && (Math.abs(inputs.armPosition.getDegrees() - setpointDegrees) <= 0.005)) {
            isMovingToTarget = false;
            return;
        }

        //otherwise, follow trapezoidal profile normally
        double desiredDegreesPerSecond = trapezoidProfile.calculate(timer.get()).velocity;

        setArmDegreesPerSecond(desiredDegreesPerSecond);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        followTrapezoidProfile();
        // mechArm.setAngle();
    }
}
