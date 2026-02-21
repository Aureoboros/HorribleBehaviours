package org.firstinspires.ftc.teamcode;
// way to many imports ugh...
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class DualLocalizerMode {

// the following basically say that the robot needs to get help once there's a discrepancy of this much
    public static double DIVERGENCE_THRESHOLD_INCHES = 2.0;

    public static double DIVERGENCE_THRESHOLD_HEADING_RAD = Math.toRadians(5.0);

    public static int DIVERGENCE_CONFIRM_TICKS = 10;


    private final TwoDeadWheelLocalizer deadWheelLocalizer;
    private final MecanumDrive.DriveLocalizer mecanumLocalizer;

    private boolean healthy = true;
    private int divergenceCounter = 0;

    // Cached poses from last update
    private Pose2d deadWheelPose;
    private Pose2d mecanumPose;

    // Last position divergence magnitude (inches), useful for telemetry
    private double lastPosDivergenceInches = 0.0;
    private double lastHeadingDivergenceRad = 0.0;

    public FusedLocalizer(
            TwoDeadWheelLocalizer deadWheelLocalizer,
            MecanumDrive.DriveLocalizer mecanumLocalizer,
            Pose2d initialPose
    ) {
        this.deadWheelLocalizer = deadWheelLocalizer;
        this.mecanumLocalizer   = mecanumLocalizer;

        // Sync both to the same starting pose
        deadWheelLocalizer.setPose(initialPose);
        mecanumLocalizer.setPose(initialPose);

        deadWheelPose = initialPose;
        mecanumPose   = initialPose;
    }

    public PoseVelocity2d update() {
        // Run both localizers every tick
        PoseVelocity2d primaryVel = deadWheelLocalizer.update();
        mecanumLocalizer.update();

        deadWheelPose = deadWheelLocalizer.getPose();
        mecanumPose   = mecanumLocalizer.getPose();

        // --- Compute disagreement ---
        double dx = deadWheelPose.position.x - mecanumPose.position.x;
        double dy = deadWheelPose.position.y - mecanumPose.position.y;
        lastPosDivergenceInches    = Math.hypot(dx, dy);
        lastHeadingDivergenceRad   = Math.abs(headingDiff(deadWheelPose, mecanumPose));

        boolean divergingNow =
                lastPosDivergenceInches  > DIVERGENCE_THRESHOLD_INCHES ||
                lastHeadingDivergenceRad > DIVERGENCE_THRESHOLD_HEADING_RAD;

        if (divergingNow) {
            divergenceCounter++;
            if (divergenceCounter >= DIVERGENCE_CONFIRM_TICKS) {
                healthy = false;
            }
        } else {
            // Agreement: decay the counter so brief spikes don't latch
            divergenceCounter = Math.max(0, divergenceCounter - 1);
            if (divergenceCounter == 0) {
                healthy = true;
            }
        }

        return primaryVel;
    }


    /** True when both localizers agree within thresholds (after hysteresis). */
    public boolean isLocalizationHealthy() {
        return healthy;
    }

    /** Positional disagreement from the last update(), in inches. */
    public double getPositionDivergenceInches() {
        return lastPosDivergenceInches;
    }

    /** Heading disagreement from the last update(), in radians. */
    public double getHeadingDivergenceRad() {
        return lastHeadingDivergenceRad;
    }

    /** How many consecutive ticks of divergence have been seen. */
    public int getDivergenceCounter() {
        return divergenceCounter;
    }


    public Pose2d getPose() {
        return deadWheelPose;
    }

    /** Raw dead wheel pose (same as getPose(), exposed for telemetry). */
    public Pose2d getDeadWheelPose() {
        return deadWheelPose;
    }

    /** Raw mecanum pose (secondary, for comparison / telemetry). */
    public Pose2d getMecanumPose() {
        return mecanumPose;
    }


    public void resetToDeadWheels() {
        Pose2d anchor = deadWheelLocalizer.getPose();
        mecanumLocalizer.setPose(anchor);
        clearDivergence();
    }


    public void resetToMecanum() {
        Pose2d anchor = mecanumLocalizer.getPose();
        deadWheelLocalizer.setPose(anchor);
        clearDivergence();
    }

    public void resetToPose(Pose2d knownPose) {
        deadWheelLocalizer.setPose(knownPose);
        mecanumLocalizer.setPose(knownPose);
        deadWheelPose = knownPose;
        mecanumPose   = knownPose;
        clearDivergence();
    }


    private void clearDivergence() {
        divergenceCounter = 0;
        healthy = true;
        lastPosDivergenceInches  = 0.0;
        lastHeadingDivergenceRad = 0.0;
    }

    private double headingDiff(Pose2d a, Pose2d b) {
        double diff = a.heading.toDouble() - b.heading.toDouble();
        // Wrap to (-pi, pi]
        while (diff >  Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}
