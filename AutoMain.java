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

import java.sql.Driver;

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
public class AutoMain extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime(); // inits runtime
    //Inits VuForia
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    
    //Where the robot starts in the field. Refer to Google docs or printed
    //file (if any).
    public int posInField = 1;
    public int team = Math.abs(posInField%2-1);
    //returns 0 for blue, 1 for red
    
    //Color sensor for glyphs
    ColorSensor glyphSensor;
    private Gyroscope imu;
    private DcMotor back_right;
    private DcMotor back_left;
    private Servo armServo;
    private NormalizedColorSensor armColorSensor;
    //Inits servo vars
    static final double              INCREMENT   =  0.01;     // amount to slew servo each CYCLE_MS cycle
    static final    int              CYCLE_MS    =    50;     // period of each cycle
    static final double              MAX_POS     =   0.5;     // Maximum rotational position
    static final double              MIN_POS     =   0.0;     // Minimum rotational position
    double armServoPosition = MAX_POS;
    //Inits motor vars
    static final double     COUNTS_PER_MOTOR_REV    =  538 ;
    static final double     DRIVE_GEAR_REDUCTION    =  1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   =  4.0 ;
    static final double     COUNTS_PER_INCH         =  (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double     DRIVE_SPEED             =  0.3;
    static final double     TURN_SPEED              =  0.1;
    static final double     TURN_AROUND             =  36.72;
    static final double     TURN_90_DEG             =  18.36;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo glyph_right = null;
    private Servo glyph_left = null;
    private double lp = 0.8;
    private double rp = 0;
    private double rPosition = 0.32;
    private double lPosition = 0.82;
    
    boolean rampUp = true;
    boolean rampDown = true;
    boolean isGlyphThere = false;
    

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
        relicTemplate.setName("relicVuMarkTemplate"); //Can help in debugging; otherwise not necessary
        
        //Inits gyroscope
        imu = hardwareMap.get(Gyroscope.class, "imu");
        
        //Configure drive motors
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Configure glyph servos
        glyph_left = hardwareMap.get(Servo.class, "glyphLeft");
        glyph_right = hardwareMap.get(Servo.class, "glyphRight");
        glyph_right.setPosition(rPosition);
        glyph_left.setPosition(lPosition);
        
        //Configure arm servo
        armServo = hardwareMap.get(Servo.class, "armServo");
        armColorSensor = hardwareMap.get(NormalizedColorSensor.class, "armColorSensor");
        if (armColorSensor instanceof SwitchableLight) {
            ((SwitchableLight)armColorSensor).enableLight(true);
        }
        
        //Inits the glyph sensor
        glyphSensor = hardwareMap.get(ColorSensor.class, "glyph_sensor");
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    //Resets the encoders of the left drive.
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   //Resets the encoders of the right drive

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            //Close glyph grabber to grab pre-loaded glyph
            glyphGrabber(false);
            
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
            armServo.setPosition(0.7);
            sleep(500);
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
                if ((colors.red > colors.blue) && (colors.red > 0.5)) {
                    if (posInField == 1) {
                        cw();
                    } else if (posInField == 2) {
                        ccw();
                    } else if (posInField == 3) {
                        cw();
                    } else if (posInField == 4) {
                        ccw();
                    }
                } else if ((colors.red < colors.blue) && (colors.blue > 0.5)) {
                    if (posInField == 1) {
                        ccw();
                    } else if (posInField == 2) {
                        cw();
                    } else if (posInField == 3) {
                        ccw();
                    } else if (posInField == 4) {
                        cw();
                    }
                } else {
                    //continue to try again
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
            armServo.setPosition(0.2);
            sleep(500);
            //Turn off light to save battery
            if (armColorSensor instanceof SwitchableLight) {
                ((SwitchableLight)armColorSensor).enableLight(false);
            }

            if (posInField == 1) {
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    //Left Code
                    encoderDrive(TURN_SPEED, 3.8, -3.8, 2.0);
                    encoderDrive(DRIVE_SPEED, 50, 50, 7.0);
                    encoderDrive(TURN_SPEED, -3.8, 3.8, 2.0);
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    //Center Code
                    encoderDrive(TURN_SPEED, 4.15, -4.15, 2.0);
                    encoderDrive(DRIVE_SPEED, 52, 52, 7.0);
                    encoderDrive(TURN_SPEED, -4.15, 4.15, 2.0);
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    //Right Code
                    encoderDrive(TURN_SPEED, 4.5, -4.5, 2.0);
                    encoderDrive(DRIVE_SPEED, 54, 54, 7.0);
                    encoderDrive(TURN_SPEED, -4.5, 4.5, 2.0);
                }
                glyphGrabber(false);
                encoderDrive(DRIVE_SPEED, -8, -8, 3.0);
                glyphGrabber(true);
                encoderDrive(DRIVE_SPEED, 10, 10, 4.0);
            } else if (posInField == 2) {
                encoderDrive(TURN_SPEED, -TURN_90_DEG, TURN_90_DEG, 5.0);
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    //Left Code
                    encoderDrive(TURN_SPEED, -4.5, 4.5, 2.0);
                    encoderDrive(DRIVE_SPEED, 54, 54, 7.0);
                    encoderDrive(TURN_SPEED, 4.5, -4.5, 2.0);
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    //Center Code
                    encoderDrive(TURN_SPEED, -4.15, 4.15, 2.0);
                    encoderDrive(DRIVE_SPEED, 52, 52, 7.0);
                    encoderDrive(TURN_SPEED, 4.15, -4.15, 2.0);
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    //Right Code
                    encoderDrive(TURN_SPEED, -3.8, 3.8, 2.0);
                    encoderDrive(DRIVE_SPEED, 50, 50, 7.0);
                    encoderDrive(TURN_SPEED, 3.8, -3.8, 2.0);
                }
                glyphGrabber(false);
                encoderDrive(DRIVE_SPEED, -8, -8, 3.0);
                glyphGrabber(true);
                encoderDrive(DRIVE_SPEED, 10, 10, 4.0);
            } else if (posInField == 3) {
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    //Left Code
                    encoderDrive(DRIVE_SPEED, 30, 30, 4.0);
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    //Center Code
                    encoderDrive(DRIVE_SPEED, 38, 38, 4.0);
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    //Right Code
                    encoderDrive(DRIVE_SPEED, 46, 46, 4.0);
                }
                encoderDrive(TURN_SPEED, -TURN_90_DEG, TURN_90_DEG, 2.4);
                encoderDrive(DRIVE_SPEED, 6, 6, 1.0);
                glyphGrabber(true);
                encoderDrive(DRIVE_SPEED, -8, -8, 1.5);
                glyphGrabber(false);
                encoderDrive(DRIVE_SPEED, 10, 10, 1.9);
            } else if (posInField == 4) {
                encoderDrive(TURN_SPEED, TURN_AROUND, -TURN_AROUND, 2.0);
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    //Left Code
                    encoderDrive(DRIVE_SPEED, 46, 46, 4.0);
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    //Center Code
                    encoderDrive(DRIVE_SPEED, 38, 38, 4.0);
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    //Right Code
                    encoderDrive(DRIVE_SPEED, 30, 30, 4.0);
                }
                encoderDrive(TURN_SPEED, -TURN_90_DEG, TURN_90_DEG, 2.4);
                encoderDrive(DRIVE_SPEED, 6, 6, 1.0);
                glyphGrabber(true);
                encoderDrive(DRIVE_SPEED, -8, -8, 1.5);
                glyphGrabber(false);
                encoderDrive(DRIVE_SPEED, 10, 10, 1.9);
            } else {
                //This should never ever happen. Stop the program
                break;
            }

    }
    }

        public void glyphGrabber(boolean open) {
            if (open) {
                rPosition = 0.60;
                lPosition = 1.00;
                glyph_left.setPosition(lPosition);
                glyph_right.setPosition(rPosition);
            } else if (!open) {
                rPosition = 0.50;
                lPosition = 0.92;
                glyph_left.setPosition(lPosition);
                glyph_right.setPosition(rPosition);
            } else {
                //Do nothing
            }
        }

        public void ccw() {
            //Turn ccw
            encoderDrive(TURN_SPEED, -TURN_90_DEG/3, TURN_90_DEG/3, 2.0);
            //And turn back
            encoderDrive(TURN_SPEED, TURN_90_DEG/3, -TURN_90_DEG/3, 2.0);
        }

        public void cw() {
            //Turn cw
            encoderDrive(TURN_SPEED, TURN_90_DEG/3, -TURN_90_DEG/3, 2.0);
            //And turn back
            encoderDrive(TURN_SPEED, -TURN_90_DEG/3, TURN_90_DEG/3, 2.0);
        }

       
        public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
            int newLeftTarget;
            int newRightTarget;
            double startingRuntime = runtime.seconds();
    
            // Ensure that the opmode is still active
            if (opModeIsActive()) {
    
                // Determine new target position, and pass to motor controller
                newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                leftDrive.setTargetPosition(newLeftTarget);
                rightDrive.setTargetPosition(newRightTarget);
    
                // Turn On RUN_TO_POSITION
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
                //Start motion.
                leftDrive.setPower(Math.abs(speed));
                rightDrive.setPower(Math.abs(speed));
    
                if (timeoutS == 0.0) {
                    while (opModeIsActive()
                    && (leftDrive.isBusy() && rightDrive.isBusy())) {
                        // Display it for the driver.
                        telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                        telemetry.addData("Path2",  "Running at %7d :%7d",
                                                    leftDrive.getCurrentPosition(),
                                                    rightDrive.getCurrentPosition());
                        telemetry.update();
                    }
                } else {
                    while (opModeIsActive()
                    && (leftDrive.isBusy() && rightDrive.isBusy())
                    && (timeoutS < runtime.seconds()-startingRuntime)) {
                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                    telemetry.addData("Path2",  "Running at %7d :%7d",
                                                leftDrive.getCurrentPosition(),
                                                rightDrive.getCurrentPosition());
                    telemetry.update();
                    }
                }
                
        }
    }
    
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
