package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Main Loop!")
public class OpMode extends LinearOpMode {

    public DcMotor left_motor, right_motor;

    

    @Override
    public void runOpMode() throws InterruptedException {

        double iL = 0.0, iR = 0.0;

        left_motor = hardwareMap.get(DcMotor.class, "fl");
        right_motor = hardwareMap.get(DcMotor.class, "fr");

        //left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        int loc = motor.getCurrentPosition();


        waitForStart();

        while (opModeIsActive())
        {
//            iL = gamepad1.left_stick_y * -0.4;
//            iR = gamepad1.right_stick_y * -0.4;
            left_motor.setPower(gamepad1.left_stick_y);
            right_motor.setPower(gamepad1.right_stick_y);
            if (Math.abs(gamepad1.left_stick_y)<=0.1){
                left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                left_motor.setPower(iL);
                iL *= 0.7;
            }
            if (Math.abs(gamepad1.right_stick_y)<=0.1){
                right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                right_motor.setPower(iR);
                iR *= 0.7;
            }
            telemetry.addData("Power: ", String.format("%f, %f", left_motor.getPower(), right_motor.getPower()));
            telemetry.update();
            sleep(50);
        }



    }
}