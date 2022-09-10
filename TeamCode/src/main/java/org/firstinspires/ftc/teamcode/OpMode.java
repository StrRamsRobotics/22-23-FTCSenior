package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Main Loop!")
public class OpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // initial setup for opmode
        telemetry.addData("Waiting for start: ", "Press start!");
        telemetry.update();

        // wait for start
        waitForStart();
        while (opModeIsActive())
        {


            telemetry.update();
        }


    }
}