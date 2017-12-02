/*
Copyright 2017 FIRST Tech Challenge Team 11792

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous
public class Auto extends LinearOpMode {
    //Inits VuForia
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    
    //Where the robot starts in the field. Refer to Google docs or printed
    //file (if any).
    public int posInField = 1;
    
    private Gyroscope imu;
    private DcMotor back_right;
    private DcMotor back_left;
    private Servo armServo;
    private NormalizedColorSensor armColorSensor;
    //Inits servo vars
    static final double              ARMINCREMENT   =  0.01;     // amount to slew servo each CYCLE_MS cycle
    static final    int              ARMCYCLE_MS    =    50;     // period of each cycle
    static final double              ARMMAX_POS     =   0.5;     // Maximum rotational position
    static final double              ARMMIN_POS     =   0.0;     // Minimum rotational position
    double armServoPosition = ARMMAX_POS;
    //Inits motor vars
    static final double     COUNTS_PER_MOTOR_REV    =  538 ;
    static final double     DRIVE_GEAR_REDUCTION    =  1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   =  4.0 ;
    static final double     COUNTS_PER_INCH         =  (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415926535897932384626433832795028);
    static final double     DRIVE_SPEED             =  0.6;
    static final double     TURN_SPEED              =  0.5;
    static final double     TURN_AROUND             =  28.274333882308139146163790449516;
    static final double     TURN_90_DEG             =  14.137166941154069573081895224758;
    
    boolean rampUp = true;
    boolean rampDown = true;


    @Override
    public void runOpMode() {
        //Continues to inti VuForia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Aazdgn3/////AAAAGWj/wwSCikKhql6EpveSzXSD0H1p2+9y3wFZTeGaDPArhcfj5j63s3TJs5sanBeaBo0JyvTQlkOvM7JJC04G9r9N7Sp7KP10vKGjvRLHpt+zpMoQX8bsKinccU0A3jMDZOBzuhn1FYS0ekhb7d1DkC1iHBz9A3vq6cdBWCBH5o2tuxkoNsSmO9j5Q8sVIKk/6HSrbaiPug78kX30DYb4cgChGfc99wx4SfkEfwuT0MN+g89ZUOgC1y4D67MZs1EfMWIMLdSdKJM9f0KReS+kqedFVSaj1gEPHGH24E4jXhbVJ7qRYrN+p6CCb52Px/8Qkyvl+Q8Sv/QBEyZHiC4p9T3chDEpH3DDFywl/qdRUPT6";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        
        //Inits gyroscope
        imu = hardwareMap.get(Gyroscope.class, "imu");
        
        //Configure drive motors
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        armServo = hardwareMap.get(Servo.class, "armServo");
        armColorSensor = hardwareMap.get(NormalizedColorSensor.class, "armColorSensor");
        if (armColorSensor instanceof SwitchableLight) {
            ((SwitchableLight)armColorSensor).enableLight(true);
        }
        
        //what team I am on:
        int team = 0;
        if (posInField % 2 == 0) {
            team = 1; //1 is red, 2 is blue
        } else if (posInField % 2 == 0) {
            team = 2; //1 is red, 2 is blue
        } else {
            //nothing.
        }
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    //Resets the encoders of the left drive.
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   //Resets the encoders of the right drive

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            //Start timer
            (new Timer()).start();
            
            
            //Vuforia
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
            //End VuForia
            
            
            
            //Move the arm servo down
            double position = 0;
            while (rampUp) {
                // Keep stepping up until we hit the max value.
                position += ARMINCREMENT;
                armServo.setPosition(position);
                sleep(ARMCYCLE_MS);
                if (position >= ARMMAX_POS ) {
                    position = ARMMAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            int exit = 0;
            //Reads the color sensor, loops until it finds if the color is red or blue,
            //and knocks off the ball that does not belong
            while (exit==0) {
                NormalizedRGBA colors = armColorSensor.getNormalizedColors();
                int color = colors.toColor();
                //Convert color to something (some) humans can understand.
                float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
                colors.red   /= max;
                colors.green /= max;
                colors.blue  /= max;
                //Finds if it is blue or red:
                if ((colors.red > colors.blue) && (colors.red > 0.7)) {
                    if (posInField == 1) {
                        //Turn ccw
                        encoderDrive(TURN_SPEED, -TURN_90_DEG, TURN_90_DEG, 2.0);
                        //And turn back
                        encoderDrive(TURN_SPEED, TURN_90_DEG, -TURN_90_DEG, 2.0);
                    } else if (posInField == 2) {
                        //Turn ccw
                        encoderDrive(TURN_SPEED, -TURN_90_DEG, TURN_90_DEG, 2.0);
                        //And turn back
                        encoderDrive(TURN_SPEED, TURN_90_DEG, -TURN_90_DEG, 2.0);
                    } else if (posInField == 3) {
                        //Turn cw
                        encoderDrive(TURN_SPEED, TURN_90_DEG, -TURN_90_DEG, 2.0);
                        //And turn back
                        encoderDrive(TURN_SPEED, -TURN_90_DEG, TURN_90_DEG, 2.0);
                    } else if (posInField == 4) {
                        //Turn cw
                        encoderDrive(TURN_SPEED, TURN_90_DEG, -TURN_90_DEG, 2.0);
                        //And turn back
                        encoderDrive(TURN_SPEED, -TURN_90_DEG, TURN_90_DEG, 2.0);
                    }
                } else if ((colors.red < colors.blue) && (colors.blue > 0.7)) {
                    if (posInField == 1) {
                        //Turn cw
                        encoderDrive(TURN_SPEED, TURN_90_DEG, -TURN_90_DEG, 2.0);
                    } else if (posInField == 2) {
                        //Turn cw
                        encoderDrive(TURN_SPEED, TURN_90_DEG, -TURN_90_DEG, 2.0);
                    } else if (posInField == 3) {
                        //Turn ccw
                        encoderDrive(TURN_SPEED, -TURN_90_DEG, TURN_90_DEG, 2.0);
                        //And turn back
                        encoderDrive(TURN_SPEED, TURN_90_DEG, -TURN_90_DEG, 2.0);
                    } else if (posInField == 4) {
                        //Turn ccw
                        encoderDrive(TURN_SPEED, -TURN_90_DEG, TURN_90_DEG, 2.0);
                        //And turn back
                        encoderDrive(TURN_SPEED, TURN_90_DEG, -TURN_90_DEG, 2.0);
                    }
                } else {
                    //try again
                }
                color = colors.toColor();
                //Show color on display:
                telemetry.addLine("normalized color:  ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));
                telemetry.update();
            }
           
            
            //Move the arm back up
            while (rampDown) {
                // Keep stepping up until we hit the max value.
                position -= ARMINCREMENT;
                armServo.setPosition(position);
                sleep(ARMCYCLE_MS);
                if (position >= ARMMAX_POS ) {
                    position = ARMMAX_POS;
                    rampDown = !rampDown;   // Switch ramp direction
                }
            }

            if (posInField == 1) {
                encoderDrive(TURN_SPEED, TURN_AROUND, -TURN_AROUND, 10.0); //Turn around
                encoderDrive(DRIVE_SPEED, TURN_AROUND, TURN_AROUND, 10.0); //Go off of the balancing stone
                encoderDrive(TURN_SPEED, TURN_90_DEG, -TURN_90_DEG, 5.0); //Turn to face the parking zone
                encoderDrive(DRIVE_SPEED, 18, 18, 7.0); //Goes into the parking zone
                sleep(1); //Park for one (1) second
                encoderDrive(DRIVE_SPEED, TURN_AROUND, TURN_AROUND, 10.0); //Goes out of the parking zone
                encoderDrive(TURN_SPEED, TURN_90_DEG, -TURN_90_DEG, 6.0); //Turns to face glyph pit
                encoderDrive(DRIVE_SPEED, 48, 48, 20.0); //Goes to glyph pit
                
            } else if (posInField == 2) {
                encoderDrive(TURN_SPEED, TURN_AROUND, -TURN_AROUND, 10.0); //Turn around
                encoderDrive(DRIVE_SPEED, TURN_AROUND, TURN_AROUND, 10.0); //Go off of the balancing stone
                encoderDrive(TURN_SPEED, -TURN_90_DEG, TURN_90_DEG, 5.0); //Turn to face the parking zone
                encoderDrive(DRIVE_SPEED, 18, 18, 7.0); //Goes into the parking zone
                sleep(1); //Park for one (1) second
                encoderDrive(DRIVE_SPEED, TURN_AROUND, TURN_AROUND, 10.0); //Goes out of the parking zone
                encoderDrive(TURN_SPEED, -TURN_90_DEG, TURN_90_DEG, 6.0); //Turns to face glyph pit
                encoderDrive(DRIVE_SPEED, 48, 48, 20.0); //Goes to glyph pit
            } else if (posInField == 3) {
                encoderDrive(DRIVE_SPEED, TURN_AROUND, TURN_AROUND, 10.0); //Move off of the balancing stone
                encoderDrive(TURN_SPEED, -TURN_90_DEG, TURN_90_DEG, 10.0); //Turns to face the parking zone
                encoderDrive(DRIVE_SPEED, TURN_AROUND, TURN_AROUND, 10.0); //Goes into the parking zone
                sleep(1); //Park for one (1) second
                encoderDrive(TURN_SPEED, TURN_AROUND, -TURN_AROUND, 10.0); //Turns around
                encoderDrive(DRIVE_SPEED, 36, 36, 10.0); //Goes to glyph pit 
            } else if (posInField == 4) {
                encoderDrive(DRIVE_SPEED, TURN_AROUND, TURN_AROUND, 10.0); //Move off of the balancing stone
                encoderDrive(TURN_SPEED, TURN_90_DEG, -TURN_90_DEG, 10.0); //Turns to face the parking zone
                encoderDrive(DRIVE_SPEED, TURN_AROUND, TURN_AROUND, 10.0); //Goes into the parking zone
                sleep(1); //Park for one (1) second
                encoderDrive(TURN_SPEED, TURN_AROUND, -TURN_AROUND, 10.0); //Turns around
                encoderDrive(DRIVE_SPEED, 36, 36, 10.0); //Goes to glyph pit 
            } else {
                //This should never ever happen. Quit the program
                break;
            }


        }
    }
        public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
            int newLeftTarget;
            int newRightTarget;
    
            // Ensure that the opmode is still active
            if (opModeIsActive()) {
    
                // Determine new target position, and pass to motor controller
                newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                robot.leftDrive.setTargetPosition(newLeftTarget);
                robot.rightDrive.setTargetPosition(newRightTarget);
    
                // Turn On RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
                // reset the timeout time and start motion.
                runtime.reset();
                robot.leftDrive.setPower(Math.abs(speed));
                robot.rightDrive.setPower(Math.abs(speed));
    
                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                if (timeoutS == 0.0) {
                    while (opModeIsActive() && (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {
                        // Display it for the driver.
                        telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                        telemetry.addData("Path2",  "Running at %7d :%7d",
                                                    robot.leftDrive.getCurrentPosition(),
                                                    robot.rightDrive.getCurrentPosition());
                        telemetry.update();
                    }
                } else {
                    while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {
                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                    telemetry.addData("Path2",  "Running at %7d :%7d",
                                                robot.leftDrive.getCurrentPosition(),
                                                robot.rightDrive.getCurrentPosition());
                    telemetry.update();
                    }
                }
                
        }
    }
    
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}

public void Timer {
    int time_passed;
    while (opModeIsActive) {
        Thread.sleep(1000);
        time_passed += 1;
    }
}