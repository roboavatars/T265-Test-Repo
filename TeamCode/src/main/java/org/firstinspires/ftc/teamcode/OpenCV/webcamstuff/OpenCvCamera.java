package org.firstinspires.ftc.teamcode.OpenCV.webcamstuff;

public interface OpenCvCamera
{
    /***
     * Open the connection to the camera device. If the camera is
     * already open, this will not do anything.
     *
     * You must call this before calling:
     * {@link #startStreaming(int, int)}
     * or {@link #startStreaming(int, int, OpenCvCameraRotation)}
     * or {@link #stopStreaming()}
     *
     * See {@link #closeCameraDevice()}
     */
    void openCameraDevice();

    /***
     * Close the connection to the camera device. If the camera is
     * already closed, this will not do anything.
     */
    void closeCameraDevice();

    /***
     * If a viewport container ID was passed to the constructor of
     * the implementing class, this method will "pause" the viewport
     * rendering thread. This can reduce CPU, memory, and power load.
     * For instance, this could be useful if you wish to see the live
     * camera preview as you are initializing your robot, but you no
     * longer require the live preview after you have finished your
     * initialization process. See {@link #resumeViewport()}
     */
    void pauseViewport();

    /***
     * If a viewport container ID was passed to the constructor of
     * the implementing class, and the viewport was previously paused
     * by {@link #pauseViewport()}, this method will "unpause" the
     * viewport rendering thread, so that you can see the live camera
     * feed on the screen again.
     */
    void resumeViewport();

    /***
     * Tell the camera to start streaming images to us! Note that you must make sure
     * the resolution you specify is supported by the camera. If it is not, an exception
     * will be thrown.
     *
     * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
     * supports streaming from the webcam in the uncompressed YUV image format. This means
     * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
     * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
     *
     * Also see the alternate {@link #startStreaming(int, int, OpenCvCameraRotation)} method.
     *
     * @param width the width of the resolution in which you would like the camera to stream
     * @param height the height of the resolution in which you would like the camera to stream
     */
    void startStreaming(int width, int height);

    /***
     * Same as {@link #startStreaming(int, int)} except for:
     *
     * @param rotation the rotation that the camera is being used in. This is so that
     *                 the image from the camera sensor can be rotated such that it is always
     *                 displayed with the image upright. For a front facing camera, rotation is
     *                 defined assuming the user is looking at the screen. For a rear facing camera
     *                 or a webcam, rotation is defined assuming the camera is facing away from the user.
     */
    void startStreaming(int width, int height, OpenCvCameraRotation rotation);

    /***
     * Stops streaming images from the camera (and, by extension, stops invoking your vision
     * pipeline), without closing ({@link #closeCameraDevice()}) the connection to the camera.
     */
    void stopStreaming();

    /***
     * Specify the image processing pipeline that you wish to be invoked upon receipt
     * of each frame from the camera. Note that switching pipelines on-the-fly (while
     * a streaming session is in flight) *IS* supported.
     *
     * @param pipeline the image processing pipeline that you wish to be invoked upon
     *                 receipt of each frame from the camera.
     */
    void setPipeline(OpenCvPipeline pipeline);

    /***
     * Get the number of frames that have been received from the camera and processed by
     * your pipeline since {@link #startStreaming(int, int)} was called.
     *
     * @return the number of frames that have been received from the camera and processed
     *         by your pipeline since {@link #startStreaming(int, int)} was called.
     */
    int getFrameCount();

    /***
     * Get the current frame rate of the overall system (including your pipeline as well as
     * overhead) averaged over the last 30 frames.
     *
     * @return the current frame rate of the overall system (including your pipeline as well
     *         as overhead) averaged over the last 30 frames.
     */
    float getFps();

    /***
     * Get the current execution time (in milliseconds) of your pipeline, averaged over the
     * last 30 frames.
     *
     * @return the current execution time (in milliseconds) of your pipeline, averaged
     *         over the last 30 frames.
     */
    int getPipelineTimeMs();

    /***
     * Get the current system overhead time (in milliseconds) for each frame, averaged over
     * the last 30 frames.
     *
     * @return the current system overhead time (in milliseconds) for each frame, averaged
     * over the last 30 frames
     */
    int getOverheadTimeMs();

    /***
     * Get the current total processing time (in milliseconds) for each frame (including
     * pipeline and overhead), averaged over the last 30 frames.
     *
     * @return the current total processing time (in milliseconds) for each frame (including
     *         pipeline and overhead), averaged over the last 30 frames.
     */
    int getTotalFrameTimeMs();

    /***
     * Get the current theoretically maximum frame rate that your pipeline (and overhead)
     * could achieve. This is useful for identifying whether or not your pipeline is the
     * bottleneck in the system. For instance, if {@link #getFps()} reports that the system
     * is running at 10FPS, and this method reported that your theoretical maximum FPS is
     * 12, then your pipeline is the bottleneck. Conversely, if {@link #getFps()} reported that
     * the system was running at 25FPS, and this method reported that your theoretical maximum
     * FPS is 100, then the camera would be the bottleneck.
     *
     * @return the current theoretically maximum frame rate that your pipeline (and overhead)
     *         could achieve.
     */
    int getCurrentPipelineMaxFps();
}