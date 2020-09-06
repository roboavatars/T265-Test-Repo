package org.firstinspires.ftc.teamcode.OpenCV.webcamstuff;

import org.opencv.core.Mat;

public abstract class OpenCvPipeline
{
    public abstract Mat processFrame(Mat input);
    public void onViewportTapped() {}
}