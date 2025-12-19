package org.firstinspires.ftc.teamcode.OC;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="OCRightDown")
public class OCRightDown extends LinearOpMode {
    DcMotor motor1out, motor2out;
    DcMotor motorin;
    DcMotor motorContinuu;
    Servo unghi, aruncare;

    @Override
    public void init()
    {
        unghi = hardwareMap.get(Servo.class, "s0");
        aruncare = hardwareMap.get(Servo.class, "s1");

        motor1out = hardwareMap.get(DcMotor.class, "motor1out");
        motor2out = hardwareMap.get(DcMotor.class, "motor2out");
        motorin = hardwareMap.get(DcMotor.class, "motorin");
        motorContinuu = hardwareMap.get(DcMotor.class, "motorcontinuu");
    }

    @Override
    public void loop() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0); // 1. Set Starting Position
        drive.setPoseEstimate(startPose); // 2. Set Initial Pose

        // 3. Build a Trajectory
        Trajectory myTrajectory = new TrajectoryBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(10, 10, Math.toRadians(90)), Math.toRadians(0))
                .build();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
    }
}
