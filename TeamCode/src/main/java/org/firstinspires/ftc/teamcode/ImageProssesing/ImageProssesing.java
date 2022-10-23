package org.firstinspires.ftc.teamcode.ImageProssesing;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

// github test

public class  ImageProssesing {

    String Tag = "Apollo Image Prossesing";

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    public enum ImageProssesingLabels{
        Bolt,
        Bulb,
        Panel
    }

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AUQVSkT/////AAABmfHYQ2uBTENAsqKk1tAS5aVHjgStefENQZMSaAGKRKVrclqNHvVYMBdviHhyzjElpfw4H1R2dfE6emqrjafzfkWe++ciBN9sWguAefU4YMIKJunggNWVvMqLVqp8V2n6VOQfn+Ec2S6yUs9Z/Pk46eF1TcDcFhvK6AKklQPHizBBu5vyixMBFvu8rp865ZC177i8mrIprQO7RJ/iCeP13hvqRKRC/dwh+71QvFoI//2fk+vdDE6OHm6pYInXBQcOmNQgjFN6L9AEh4uMM1yJtOEfm7EtwDdzrZO3OeLwTre7BoZz3HyfwWdco0t3s4hXLq2BqUCmkrYeOlFQbR0emv844IEr58ZX7FOoGl5VSg1D";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
    public void ImageprossesingInit(HardwareMap hardwareMapImageProssesing) {


        initVuforia(hardwareMapImageProssesing);
        initTfod(hardwareMapImageProssesing);

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);

            Log.d(Tag, "> finish Init");

        }
    }

    public ImageProssesingLabels ImageProssesingProsses() {
        ImageProssesingLabels imageProssesingLabels = null;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {

                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());

                    Log.d(Tag, "colom = " + col );
                    Log.d(Tag, "row = " + row);
                    Log.d(Tag, "width = " + width);
                    Log.d(Tag, "height = " + height);

                    String lable = recognition.getLabel();

                    if(lable == "1 Bolt"){
                        Log.d(Tag, "Image = Bolt");
                        imageProssesingLabels = ImageProssesingLabels.Bolt;
                    }
                    else if(lable == "2 Bulb"){
                        Log.d(Tag, "image = Bulb");
                        imageProssesingLabels =ImageProssesingLabels.Bulb;
                    }
                    else if(lable == "3 Panel"){
                        Log.d(Tag, "image = Panel");
                        imageProssesingLabels = ImageProssesingLabels.Panel;
                    }

                }
            }
        }
        return (imageProssesingLabels);
    }
}
