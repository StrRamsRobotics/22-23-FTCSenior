package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.wrappers.MotorWrapper;

@TeleOp(name="Main Loop!")
public class OpMode extends LinearOpMode {

    public MotorWrapper mleft, mright;
    public DcMotor geartest, arm;

    public Servo servoTest;

    @Override
    public void runOpMode() throws InterruptedException {

        mleft = new MotorWrapper(hardwareMap.get(DcMotor.class, "fl"));
        mright = new MotorWrapper(hardwareMap.get(DcMotor.class, "fr"));
        arm = hardwareMap.get(DcMotor.class, "arm");

        mleft.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        // geartest
        geartest = hardwareMap.get(DcMotor.class, "geartest");
        geartest.setPower(1.0);

//        int loc = motor.getCurrentPosition();

        // servo testing
        servoTest = hardwareMap.get(Servo.class, "servo");


        waitForStart();

        while (opModeIsActive()) {
            // movement code
            double power, turn_power;
            if (Math.abs(gamepad1.left_stick_y)<=0.1) power = 0;
            else power = gamepad1.left_stick_y;
            if (Math.abs(gamepad1.right_stick_x)<=0.1) turn_power = 0;
            else turn_power = gamepad1.right_stick_x;
            // set power
            mleft.getMotor().setPower(power+turn_power);
            mright.getMotor().setPower(power-turn_power);

            // servo action


            // add telemetry
            telemetry.addData("Power: ", String.format("%d, %d", mright.getCurrentTicks(), mright.getCurrentTicks()));
            telemetry.update();
            sleep(50);
        }



    }
}