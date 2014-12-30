#include "cv.h"
#include "highgui.h"

int main()
{
  int i;
  CvCapture* capture = 0;
  capture = cvCreateCameraCapture(2);
  if(!capture){
  return -1;
  }
 IplImage *bgr_frame=cvQueryFrame(capture);//Init the video read

 CvSize size = cvSize(
   (int)cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH),
   (int)cvGetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT)
   );
 IplImage *img=cvCreateImage(size,8,1);
 CvVideoWriter *writer = cvCreateVideoWriter
 (
  "siddhesh.avi",
   CV_FOURCC('P','I','M','1'),
   10,
   size,
true
 );
  for(i=0;i<100;i++)
  {
    bgr_frame = cvQueryFrame(capture);
    //cvCvtColor(bgr_frame,img,CV_BGR2GRAY);
    cvWriteFrame( writer, bgr_frame );
 
  }
cvReleaseVideoWriter( &writer );
cvReleaseImage( &img );

cvReleaseCapture( &capture );

 
  return 0;
}
