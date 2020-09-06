package org.firstinspires.ftc.teamcode.OpenCV.webcamstuff;

import android.annotation.SuppressLint;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.opencv.BuildConfig;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.ArrayBlockingQueue;

public class OpenCvViewport extends SurfaceView implements SurfaceHolder.Callback
{
    private Size size;
    private Bitmap bitmapFromMat;
    private RenderThread renderThread;
    private Canvas canvas = null;
    private double aspectRatio;
    private static final int VISION_PREVIEW_FRAME_QUEUE_CAPACITY = 2;
    private static final int FRAMEBUFFER_RECYCLER_CAPACITY = VISION_PREVIEW_FRAME_QUEUE_CAPACITY + 2; //So that the evicting queue can be full, and the render thread has one checked out (+1) and post() can still take one (+1).
    private EvictingBlockingQueue<MatRecycler.RecyclableMat> visionPreviewFrameQueue = new EvictingBlockingQueue<>(new ArrayBlockingQueue<MatRecycler.RecyclableMat>(VISION_PREVIEW_FRAME_QUEUE_CAPACITY));
    private MatRecycler framebufferRecycler;
    private volatile RenderingState internalRenderingState = RenderingState.STOPPED;
    private final Object syncObj = new Object();
    private volatile boolean userRequestedActive = false;
    private volatile boolean userRequestedPause = false;
    private boolean needToDeactivateRegardlessOfUser = false;
    private boolean surfaceExistsAndIsReady = false;
    private float fps = 0;
    private String TAG = "OpenCvViewport";

    public OpenCvViewport(Context context, OnClickListener onClickListener)
    {
        super(context);

        getHolder().addCallback(this);

        visionPreviewFrameQueue.setEvictAction(new Consumer<MatRecycler.RecyclableMat>()
        {
            @Override
            public void accept(MatRecycler.RecyclableMat value)
            {
                /*
                 * If a Mat is evicted from the queue, we need
                 * to make sure to return it to the Mat recycler
                 */
                framebufferRecycler.returnMat(value);
            }
        });

        setOnClickListener(onClickListener);
    }

    private enum RenderingState
    {
        STOPPED,
        ACTIVE,
        PAUSED,
    }

    public void setSize(Size size)
    {
        synchronized (syncObj)
        {
            if(renderThread != null)
            {
                renderThread.interrupt();
            }

            //did they give us null?
            if(size == null)
            {
                //ugh, they did
                throw new IllegalArgumentException("size cannot be null!");
            }

            this.size = size;
            this.aspectRatio = (double)size.getWidth() / (double)size.getHeight();

            framebufferRecycler = new MatRecycler(FRAMEBUFFER_RECYCLER_CAPACITY);
        }
    }

    public void post(Mat mat)
    {
        synchronized (syncObj)
        {
            //did they give us null?
            if(mat == null)
            {
                //ugh, they did
                throw new IllegalArgumentException("cannot post null mat!");
            }

            //Are we actually rendering to the display right now? If not,
            //no need to waste time doing a memcpy
            if(internalRenderingState == RenderingState.ACTIVE)
            {
                /*
                 * We need to copy this mat before adding it to the queue,
                 * because the pointer that was passed in here is only known
                 * to be pointing to a certain frame while we're executing.
                 */
                try
                {
                    /*
                     * Grab a framebuffer Mat from the recycler
                     * instead of doing a new alloc and then having
                     * to free it after rendering/eviction from queue
                     */
                    MatRecycler.RecyclableMat matToCopyTo = framebufferRecycler.takeMat();
                    mat.copyTo(matToCopyTo);
                    visionPreviewFrameQueue.offer(matToCopyTo);
                }
                catch (InterruptedException e)
                {
                    e.printStackTrace();
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    /*
     * Called with syncObj held
     */
    public void checkState()
    {
        /*
         * If the surface isn't ready, don't do anything
         */
        if(!surfaceExistsAndIsReady)
        {
            Log.d(TAG, "CheckState(): surface not ready or doesn't exist");
            return;
        }

        /*
         * Does the user want us to stop?
         */
        if(!userRequestedActive || needToDeactivateRegardlessOfUser)
        {
            Log.d(TAG, "CheckState(): user requested that we deactivate");

            /*
             * We only need to stop the render thread if it's not
             * already stopped
             */
            if(internalRenderingState != RenderingState.STOPPED)
            {
                Log.d(TAG, "CheckState(): deactivating viewport");

                /*
                 * Interrupt him so he's not stuck looking at his
                 * frame queue.
                 */
                renderThread.notifyExitRequested();
                renderThread.interrupt();

                try
                {
                    /*
                     * Wait for him to die
                     */
                    renderThread.join();
                }
                catch (InterruptedException e)
                {
                    e.printStackTrace();
                }

                internalRenderingState = RenderingState.STOPPED;
            }
            else
            {
                Log.d(TAG, "CheckState(): already deactivated");
            }
        }

        /*
         * Does the user want us to start?
         */
        else if(userRequestedActive)
        {
            Log.d(TAG, "CheckState(): user requested that we activate");

            /*
             * We only need to start the render thread if it's
             * stopped.
             */
            if(internalRenderingState == RenderingState.STOPPED)
            {
                Log.d(TAG, "CheckState(): activating viewport");

                internalRenderingState = RenderingState.PAUSED;

                if(userRequestedPause)
                {
                    internalRenderingState = RenderingState.PAUSED;
                }
                else
                {
                    internalRenderingState = RenderingState.ACTIVE;
                }

                renderThread = new RenderThread();
                renderThread.start();
            }
            else
            {
                Log.d(TAG, "CheckState(): already activated");
            }
        }

        if(internalRenderingState != RenderingState.STOPPED)
        {
            if(userRequestedPause && internalRenderingState != RenderingState.PAUSED
                    || !userRequestedPause && internalRenderingState != RenderingState.ACTIVE)
            {
                if(userRequestedPause)
                {
                    Log.d(TAG, "CheckState(): pausing viewport");
                    internalRenderingState = RenderingState.PAUSED;
                }
                else
                {
                    Log.d(TAG, "CheckState(): resuming viewport");
                    internalRenderingState = RenderingState.ACTIVE;
                }

                /*
                 * Interrupt him so that he's not stuck looking at his frame queue.
                 * (We stop filling the frame queue if the user requested pause so
                 * we aren't doing pointless memcpys)
                 */
                renderThread.interrupt();
            }
        }
    }

    /***
     * Activate the render thread
     */
    public synchronized void activate()
    {
        synchronized (syncObj)
        {
            userRequestedActive = true;
            checkState();
        }
    }

    /***
     * Deactivate the render thread
     */
    public synchronized void deactivate()
    {
        synchronized (syncObj)
        {
            userRequestedActive = false;
            checkState();
        }
    }

    public synchronized void resume()
    {
        synchronized (syncObj)
        {
            userRequestedPause = false;
            checkState();
        }
    }

    public synchronized void pause()
    {
        synchronized (syncObj)
        {
            userRequestedPause = true;
            checkState();
        }
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder)
    {

    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height)
    {
        synchronized (syncObj)
        {
            needToDeactivateRegardlessOfUser = false;
            surfaceExistsAndIsReady = true;

            checkState();
        }
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder)
    {
        synchronized (syncObj)
        {
            needToDeactivateRegardlessOfUser = true;
            checkState();
            surfaceExistsAndIsReady = false;
        }

    }

    public void notifyStatistics(float fps)
    {
        this.fps = fps;
    }

    class RenderThread extends Thread
    {
        boolean shouldPaintOrange = true;
        volatile boolean exitRequested = false;
        private String TAG = "OpenCvViewportRenderThread";

        public void notifyExitRequested()
        {
            exitRequested = true;
        }

        @Override
        public void run()
        {
            Log.d(TAG, "I am alive!");

            bitmapFromMat = Bitmap.createBitmap(size.getWidth(), size.getHeight(), Bitmap.Config.RGB_565);

            canvas = getHolder().lockCanvas();
            canvas.drawColor(Color.BLUE);
            getHolder().unlockCanvasAndPost(canvas);

            while (true)
            {
                /*
                 * Do we need to exit?
                 */
                if(exitRequested)
                {
                    break;
                }

                switch (internalRenderingState)
                {
                    case ACTIVE:
                    {
                        shouldPaintOrange = true;

                        MatRecycler.RecyclableMat mat;

                        try
                        {
                            //Grab a Mat from the frame queue
                            mat = visionPreviewFrameQueue.take();
                        }
                        catch (InterruptedException e)
                        {
                            e.printStackTrace();
                            break;
                        }

                        //Get canvas object for rendering on
                        canvas = getHolder().lockCanvas();

                        /*
                         * For some reason, the canvas will very occasionally be null upon closing.
                         * Stack Overflow seems to suggest this means the canvas has been destroyed.
                         * However, surfaceDestroyed(), which is called right before the surface is
                         * destroyed, calls checkState(), which *SHOULD* block until we die. This
                         * works most of the time, but not always? We don't yet understand...
                         */
                        if(canvas != null)
                        {
                            //Convert that Mat to a bitmap we can render
                            Utils.matToBitmap(mat, bitmapFromMat);

                            //Draw the background black each time to prevent double buffering problems
                            canvas.drawColor(Color.BLACK);

                            //Landscape
                            if((canvas.getHeight() * aspectRatio) < canvas.getWidth())
                            {
                                //Draw the bitmap, scaling it to the maximum size that will fit in the viewport
                                canvas.drawBitmap(
                                        bitmapFromMat,
                                        new Rect(0,0,bitmapFromMat.getWidth(), bitmapFromMat.getHeight()),
                                        new Rect(0,0,(int) Math.round(canvas.getHeight() * aspectRatio), canvas.getHeight()),
                                        null);
                            }
                            //Portrait
                            else
                            {
                                //Draw the bitmap, scaling it to the maximum size that will fit in the viewport
                                canvas.drawBitmap(
                                        bitmapFromMat,
                                        new Rect(0,0,bitmapFromMat.getWidth(), bitmapFromMat.getHeight()),
                                        new Rect(0,0,canvas.getWidth(), (int) Math.round(canvas.getWidth() / aspectRatio)),
                                        null);
                            }

                            getHolder().unlockCanvasAndPost(canvas);
                        }
                        else
                        {
                            Log.d(TAG, "Canvas was null");
                        }

                        //We're done with that Mat object; return it to the Mat recycler so it can be used again later
                        framebufferRecycler.returnMat(mat);

                        break;
                    }

                    case PAUSED:
                    {
                        if(shouldPaintOrange)
                        {
                            shouldPaintOrange = false;

                            canvas = getHolder().lockCanvas();

                            /*
                             * For some reason, the canvas will very occasionally be null upon closing.
                             * Stack Overflow seems to suggest this means the canvas has been destroyed.
                             * However, surfaceDestroyed(), which is called right before the surface is
                             * destroyed, calls checkState(), which *SHOULD* block until we die. This
                             * works most of the time, but not always? We don't yet understand...
                             */
                            if(canvas != null)
                            {
                                canvas.drawColor(Color.rgb(255, 166, 0));
                                getHolder().unlockCanvasAndPost(canvas);
                            }
                        }

                        try
                        {
                            Thread.sleep(50);
                        }
                        catch (InterruptedException e)
                        {
                            e.printStackTrace();
                        }
                        break;
                    }
                }
            }

            Log.d(TAG, "About to exit");
            bitmapFromMat.recycle(); //Help the garbage collector :)
        }
    }

    @SuppressLint("DefaultLocale")
    public String getFpsString()
    {
        return String.format("FPS@%dx%d: %.2f", size.getWidth(), size.getHeight(), fps);
    }
}