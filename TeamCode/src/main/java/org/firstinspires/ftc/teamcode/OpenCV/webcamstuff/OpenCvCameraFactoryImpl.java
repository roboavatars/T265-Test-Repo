package org.firstinspires.ftc.teamcode.OpenCV.webcamstuff;

import android.content.Context;
import android.support.annotation.IdRes;
import android.view.View;
import android.view.ViewGroup;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.ArrayList;
import java.util.concurrent.CountDownLatch;

class OpenCvCameraFactoryImpl extends OpenCvCameraFactory
{
    static void init()
    {
        OpenCvCameraFactory.theInstance = new OpenCvCameraFactoryImpl();
    }

    @OpModeRegistrar
    public static void initOnSdkBoot(Context context, AnnotatedOpModeManager manager)
    {
        init();
    }

    @Override
    public OpenCvInternalCamera createInternalCamera(OpenCvInternalCamera.CameraDirection direction)
    {
        return new OpenCvInternalCameraImpl(direction);
    }

    @Override
    public OpenCvInternalCamera createInternalCamera(OpenCvInternalCamera.CameraDirection direction, int containerId)
    {
        return new OpenCvInternalCameraImpl(direction, containerId);
    }

    @Override
    public OpenCvCamera createWebcam(WebcamName webcamName)
    {
        return new OpenCvWebcamImpl(webcamName);
    }

    @Override
    public OpenCvCamera createWebcam(WebcamName webcamName, @IdRes int viewportContainerId)
    {
        return new OpenCvWebcamImpl(webcamName, viewportContainerId);
    }

    @Override
    public int[] splitLayoutForMultipleViewports(final int containerId, final int numViewports, final ViewportSplitMethod viewportSplitMethod)
    {
        if(numViewports < 2)
        {
            throw new IllegalArgumentException("Layout requested to be split for <2 viewports!");
        }

        final int[] ids = new int[numViewports];
        final ArrayList<LinearLayout> layoutArrayList = new ArrayList<>(numViewports);

        final CountDownLatch latch = new CountDownLatch(1);

        //We do the viewport creation on the UI thread, but if there's an exception then
        //we need to catch it and rethrow it on the OpMode thread
        final RuntimeException[] exToRethrowOnOpModeThread = {null};

        AppUtil.getInstance().getActivity().runOnUiThread(new Runnable()
        {
            @Override
            public void run()
            {
                try
                {
                    final LinearLayout containerLayout = (LinearLayout) AppUtil.getInstance().getActivity().findViewById(containerId);

                    if(containerLayout == null)
                    {
                        throw new OpenCvCameraException("Viewport container specified by user does not exist!");
                    }
                    else if(containerLayout.getChildCount() != 0)
                    {
                        throw new OpenCvCameraException("Viewport container specified by user is not empty!");
                    }

                    containerLayout.setVisibility(View.VISIBLE);

                    if(viewportSplitMethod == null)
                    {
                        throw new IllegalArgumentException("Viewport split method cannot be null!");
                    }
                    else if(viewportSplitMethod == ViewportSplitMethod.VERTICALLY)
                    {
                        containerLayout.setOrientation(LinearLayout.VERTICAL);
                    }
                    else
                    {
                        containerLayout.setOrientation(LinearLayout.HORIZONTAL);
                    }

                    for(int i = 0; i < numViewports; i++)
                    {
                        LinearLayout linearLayout = new LinearLayout(AppUtil.getInstance().getActivity());
                        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT);
                        params.weight = 1;
                        linearLayout.setLayoutParams(params);
                        linearLayout.setId(View.generateViewId());
                        ids[i] = linearLayout.getId();
                        layoutArrayList.add(linearLayout);
                        containerLayout.addView(linearLayout);
                    }

                    LIFO_OpModeCallbackDelegate.getInstance().add(new LIFO_OpModeCallbackDelegate.OnOpModeStoppedListener()
                    {
                        @Override
                        public void onOpModePostStop(OpMode opMode)
                        {
                            AppUtil.getInstance().getActivity().runOnUiThread(new Runnable()
                            {
                                @Override
                                public void run()
                                {
                                    for(LinearLayout layout : layoutArrayList)
                                    {
                                        containerLayout.removeView(layout);
                                    }
                                    containerLayout.setVisibility(View.GONE);
                                    containerLayout.setOrientation(LinearLayout.VERTICAL);
                                }
                            });

                        }
                    });

                    latch.countDown();
                }
                catch (RuntimeException e)
                {
                    exToRethrowOnOpModeThread[0] = e;
                }

            }
        });

        if(exToRethrowOnOpModeThread[0] != null)
        {
            throw exToRethrowOnOpModeThread[0];
        }

        try
        {
            latch.await();

            return ids;
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
            return null;
        }
    }
}