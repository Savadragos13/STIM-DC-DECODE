package org.firstinspires.ftc.teamcode.Festival;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = " Control_cu_odometrie")
public class Control_cu_odometrie extends LinearOpMode{
    DcMotor motor1out, motor2out;
    DcMotor motorin;
    DcMotor motorContinuu;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        motor1out = hardwareMap.get(DcMotor.class, "motor1out");
        motor2out = hardwareMap.get(DcMotor.class, "motor2out");
        motorin = hardwareMap.get(DcMotor.class, "motorin");
        motorContinuu = hardwareMap.get(DcMotor.class, "motorcontinuu");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            motorContinuu.setPower(0.85 );

            float mo = gamepad2.right_trigger, mi = gamepad2.left_trigger;
            if (Math.abs(mo) > 0.1) {
                motor1out.setPower(-1);
                motor2out.setPower(1);
            }
            else{
                motor1out.setPower(0);
                motor2out.setPower(0);
            }
            if (Math.abs(mi) > 0.1) {
                motorin.setPower(1);
            }
            else
                motorin.setPower(0);
        }/*leftFront rightFront leftRear rightRear*/
    }
}
