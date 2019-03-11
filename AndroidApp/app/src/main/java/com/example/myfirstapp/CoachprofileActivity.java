package com.example.myfirstapp;

import android.os.Environment;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.content.Intent;
import android.provider.MediaStore;
import android.net.Uri;
import android.util.Log;
import android.view.View;
import android.widget.MediaController;
import android.widget.VideoView;

import java.io.File;
import java.text.SimpleDateFormat;

import static android.provider.MediaStore.Files.FileColumns.MEDIA_TYPE_VIDEO;


public class CoachprofileActivity extends AppCompatActivity {
    private VideoView mVideoView;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
    }

    static final int REQUEST_VIDEO_CAPTURE = 1;
    //static final Uri locationForVideos;

    public void dispatchTakeVideoIntent(View v) {
        //first create video file .
        //val videoFile=getOutputMediaFile();
        Intent takeVideoIntent = new Intent(MediaStore.ACTION_VIDEO_CAPTURE);

        Uri videoUri = getOutputMediaFileUri(MEDIA_TYPE_VIDEO);
        takeVideoIntent.putExtra(MediaStore.EXTRA_OUTPUT, videoUri);
        takeVideoIntent.putExtra(MediaStore.EXTRA_VIDEO_QUALITY, 1);
        //Uri videoUri=getOutputMediaFileUri();
        if (takeVideoIntent.resolveActivity(getPackageManager()) != null) {
            startActivityForResult(takeVideoIntent, REQUEST_VIDEO_CAPTURE);
        }
    }

    public void dispatchEditVideoIntent(View v) {
        //first create video file .
        //val videoFile=getOutputMediaFile();
        Intent EditVideoIntent = new Intent(MediaStore.ACTION_VIDEO_CAPTURE);

    }

    private static Uri getOutputMediaFileUri(int type){
        return Uri.fromFile(getOutputMediaFile(type));
    }

    private static File getOutputMediaFile(int type){
        // Check that the SDCard is mounted
        File mediaStorageDir = new File(Environment.getExternalStorageDirectory() +"/PlayerDirectoryName");
        // Create the storage directory(MyCameraVideo) if it does not exist
        if (! mediaStorageDir.exists()){

            if (! mediaStorageDir.mkdirs()){

                Log.d("MyCameraVideo", "Failed to create directory MyCameraVideo.");
                return null;
            }
        }

        // Create a media file name
        // For unique file name appending current timeStamp with file name
        java.util.Date date= new java.util.Date();
        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(date.getTime());

        File mediaFile;
        if(type == MEDIA_TYPE_VIDEO) {

            // For unique video file name appending current timeStamp with file name
            mediaFile = new File(mediaStorageDir.getPath() + File.separator +
                    "VID_"+ timeStamp + ".mp4");

        } else {
            return null;
        }

        return mediaFile;
    }



    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent intent) {
        if (requestCode == REQUEST_VIDEO_CAPTURE && resultCode == RESULT_OK) {
            Uri videoUri = intent.getData();
            // file saved at URI specified in the intent .
            //Intent takeVideoIntent = new Intent(MediaStore.ACTION_VIDEO_CAPTURE);


            //make an intent to start the view video activity .

            mVideoView.setVideoURI(videoUri); //for viweing the recorded video.
            //add stuff to edit the view and save the video to a list navigatable from the usr's profile page.
            //File VideoFile = new File(videoUri.getPath());
        }
    }




}