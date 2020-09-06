package org.firstinspires.ftc.teamcode.OpenCV.webcamstuff;

import android.util.Log;

import org.opencv.core.Mat;

import java.util.concurrent.ArrayBlockingQueue;

/*
 * A utility class for managing the re-use of Mats
 * so as to re-use already allocated memory instead
 * of constantly allocating new Mats and then freeing
 * them after use.
 */
class MatRecycler
{
    private RecyclableMat[] mats;
    private ArrayBlockingQueue<RecyclableMat> availableMats;

    MatRecycler(int num)
    {
        mats = new RecyclableMat[num];
        availableMats = new ArrayBlockingQueue<>(num);

        for(int i = 0; i < mats.length; i++)
        {
            mats[i] = new RecyclableMat(i);
            availableMats.add(mats[i]);
        }
    }

    synchronized RecyclableMat takeMat() throws InterruptedException
    {
        if(availableMats.size() == 0)
        {
            throw new RuntimeException("All mats have been checked out!");
        }

        RecyclableMat mat = availableMats.take();
        mat.checkedOut = true;
        return mat;
    }

    synchronized void returnMat(RecyclableMat mat)
    {
        if(mat != mats[mat.idx])
        {
            throw new IllegalArgumentException("This mat does not belong to this recycler!");
        }

        if(mat.checkedOut)
        {
            mat.checkedOut = false;
            availableMats.add(mat);
        }
        else
        {
            throw new IllegalArgumentException("This mat has already been returned!");
        }
    }

    class RecyclableMat extends Mat
    {
        private int idx = -1;
        private volatile boolean checkedOut = false;

        private RecyclableMat(int idx)
        {
            this.idx = idx;
        }
    }
}