package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="hopefullyRight", group="Iterative Opmode")
public class hopefullyRight extends OpMode {
    ElapsedTime runtime = new ElapsedTime();

    DcMotor LeftFrontWheels = null;
    DcMotor LeftBackWheels = null;
    DcMotor RightFrontWheels = null;
    DcMotor RightBackWheels = null;

    DcMotor inTilt = null;
    DcMotor paddles = null;
    DcMotor outLift = null;
    CRServo inLift;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        LeftFrontWheels = hardwareMap.get(DcMotor.class,"LFW");
        RightFrontWheels = hardwareMap.get(DcMotor.class,"RFW");
        RightBackWheels = hardwareMap.get(DcMotor.class,"RBW");
        LeftBackWheels = hardwareMap.get(DcMotor.class,"LBW");

        inTilt = hardwareMap.get(DcMotor.class,"inTilt");
        paddles = hardwareMap.get(DcMotor.class,"paddles");
        outLift = hardwareMap.get(DcMotor.class,"outLift");
        inLift = hardwareMap.get(CRServo.class,"inLift");

        LeftFrontWheels.setDirection(DcMotor.Direction.REVERSE);
        LeftBackWheels.setDirection(DcMotor.Direction.REVERSE);
        RightFrontWheels.setDirection(DcMotor.Direction.FORWARD);
        RightBackWheels.setDirection(DcMotor.Direction.REVERSE);

        inTilt.setDirection(DcMotor.Direction.REVERSE);
        paddles.setDirection(DcMotor.Direction.REVERSE);
        outLift.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();

    }

    @Override
    public void loop() {

        double leftPower;
        double rightPower;
        double liftPower = 0.5;

        leftPower  = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;

        if(leftPower != 0 || rightPower != 0) {
            LeftFrontWheels.setPower(leftPower);
            LeftBackWheels.setPower(leftPower);
            RightFrontWheels.setPower(rightPower);
            RightBackWheels.setPower(rightPower);
        }
        /*
!!!!!!!!!!!!!Intake and outtake switched on configuring!!!!!!!!!!!!!!!!!
        */
        //tilting intake box up and down
        if(gamepad1.a){
           // runtime.reset();
           // while(runtime.seconds() < 2) {
                outLift.setPower(liftPower);
                inLift.setPower(-liftPower/2);
            //}
        }
        if(gamepad1.b){
            outLift.setPower(-liftPower*2);
        }

        //moving outtake box up and down
        if(gamepad1.dpad_down){
                inTilt.setPower(liftPower/3);
        }
        if(gamepad1.dpad_up){
            inTilt.setPower(-liftPower*2);
        }

        //reel in intake box
        if(gamepad1.dpad_right){
            inLift.setPower(liftPower/2);
        }

        //spin paddles
        if(gamepad1.left_bumper){
            paddles.setPower(.6);
        }



        else {
            RightFrontWheels.setPower(0);
            RightBackWheels.setPower(0);
            LeftBackWheels.setPower(0);
            LeftFrontWheels.setPower(0);
            inTilt.setPower(0);
            paddles.setPower(.6);
            outLift.setPower(0);
            //inLift.setPower(0);
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

    }

    @Override
    public void stop() {
    }

}
