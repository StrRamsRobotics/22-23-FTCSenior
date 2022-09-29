package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Main Loop!")
public class OpMode extends LinearOpMode {

    public DcMotor left_motor, right_motor, geartest, arm;
    

    @Override
    public void runOpMode() throws InterruptedException {

        left_motor = hardwareMap.get(DcMotor.class, "fl");
        right_motor = hardwareMap.get(DcMotor.class, "fr");
        arm = hardwareMap.get(DcMotor.class, "arm");

        left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // geartest
        geartest = hardwareMap.get(DcMotor.class, "geartest");
        geartest.setPower(1.0);

//        int loc = motor.getCurrentPosition();


        waitForStart();

        while (opModeIsActive()) {
            double power, turn_power;
            if (Math.abs(gamepad1.left_stick_y)<=0.1) power = 0;
            else power = gamepad1.left_stick_y;
            if (Math.abs(gamepad1.right_stick_x)<=0.1) turn_power = 0;
            else turn_power = gamepad1.right_stick_x;

            left_motor.setPower(power+turn_power);
            right_motor.setPower(power-turn_power);

            telemetry.addData("Power: ", String.format("%d, %d", left_motor.getCurrentPosition(), right_motor.getCurrentPosition()));
            telemetry.update();
            sleep(50);
        }



    }
}