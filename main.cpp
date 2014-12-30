#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _EiC
// motion templates sample code
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <string>
#include <complex>
#include <cmath>
#include <algorithm>
#include "wavelet2d.h"
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"

#endif

using namespace std;
using namespace cv;
// various tracking parameters (in seconds)
const double MHI_DURATION = 1;
const double MAX_TIME_DELTA = 0.5;
const double MIN_TIME_DELTA = 0.05;
// number of cyclic frame buffer used for motion detection
// (should, probably, depend on FPS)
const int N = 4;

// ring image buffer
IplImage **buf = 0;
int last = 0;

// temporary images
IplImage *mhi = 0; // MHI
IplImage *orient = 0; // orientation
IplImage *mask = 0; // valid orientation mask
IplImage *segmask = 0; // motion segmentation map
CvMemStorage* storage = 0; // temporary storage

int i, idx1 = last, idx2;
    IplImage* silh;
    CvSeq* seq;
    CvRect comp_rect;
    CvRect comp_rect_temp;
    CvRect comp_rect_req;    
    CvPoint comp_rect_req_center;
    double angle;
    CvPoint center;
    double magnitude1;          
    CvScalar color;
    int diff_threshold=50;
    CvPoint pt1,pt2,pt3,pt4;
    IplImage* img;
//images for wavelets
    IplImage *img_req_rgb;
    IplImage *img_req;
    IplImage *img_req_lp;
    IplImage *img_req_hp;
    IplImage *cvImg;

//variables for motors
   int no_samples;
   int no_samples_new;
   int avg_ratio;
   int avg_ratio_new;
   int motor_moving; 
   int motor_direction;   

void* maxval(vector<vector<double> > &arr, double &max){
	max = 0;
	for (unsigned int i =0; i < arr.size(); i++) {
		for (unsigned int j =0; j < arr[0].size(); j++) {
			if (max <= arr[i][j]){
				max = arr[i][j];
			}
		}
	}
	return 0;
}

void* maxval1(vector<double> &arr, double &max){
	max = 0;
	for (unsigned int i =0; i < arr.size(); i++) {
		if (max <= arr[i]){
			max = arr[i];
		}

	}
	return 0;
}




int main(int argc, char** argv)
{
    double count;double maxcount;
    int index=0;
    IplImage* motion = 0;
    CvCapture* capture = 0;
    
    no_samples=0;
    no_samples_new=0;
    avg_ratio=0;
    avg_ratio_new=0;
    motor_moving=0;
    motor_direction=0; 
      
     capture = cvCaptureFromCAM(1);
     //cvNamedWindow("From Camera",1);    

    if( capture )
    {
      //  cvNamedWindow( "Motion", 1 );
        
        for(;;)
        {
            
            if( !cvGrabFrame( capture ))
                break;
            img = cvRetrieveFrame( capture );
            
        //    cvShowImage("From Camera",img);
            if( img )
            {
                if( !motion )
                {
                    motion = cvCreateImage( cvSize(img->width,img->height), 8, 3 );
                    cvZero( motion );
                    motion->origin = img->origin;
                }
            }

  //          update_mhi( image, motion, 80 );
    double timestamp = (double)clock()/CLOCKS_PER_SEC; // get current time in seconds
    CvSize size = cvSize(img->width,img->height); // get current frame size
    

    // allocate images at the beginning or
    // reallocate them if the frame size is changed
    if( !mhi || mhi->width != size.width || mhi->height != size.height ) {
        if( buf == 0 ) {
            buf = (IplImage**)malloc(N*sizeof(buf[0]));
            memset( buf, 0, N*sizeof(buf[0]));
        }
        
        for( i = 0; i < N; i++ ) {
            cvReleaseImage( &buf[i] );
            buf[i] = cvCreateImage( size, IPL_DEPTH_8U, 1 );
            cvZero( buf[i] );
        }
        cvReleaseImage( &mhi );
        cvReleaseImage( &orient );
        cvReleaseImage( &segmask );
        cvReleaseImage( &mask );
        
        mhi = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        cvZero( mhi ); // clear MHI at the beginning
        orient = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        segmask = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        mask = cvCreateImage( size, IPL_DEPTH_8U, 1 );
    }

    cvCvtColor( img, buf[last], CV_BGR2GRAY ); // convert frame to grayscale

    idx2 = (last + 1) % N; // index of (last - (N-1))th frame
    last = idx2;

    silh = buf[idx2];
    cvAbsDiff( buf[idx1], buf[idx2], silh ); // get difference between frames
    
    cvThreshold( silh, silh, diff_threshold, 1, CV_THRESH_BINARY ); // and threshold it
    cvUpdateMotionHistory( silh, mhi, timestamp, MHI_DURATION ); // update MHI

    // convert MHI to blue 8u image
    cvCvtScale( mhi, mask, 255./MHI_DURATION,
                (MHI_DURATION - timestamp)*255./MHI_DURATION );
    cvZero( motion );
    cvCvtPlaneToPix( mask, 0, 0, 0, motion );

    // calculate motion gradient orientation and valid orientation mask
    cvCalcMotionGradient( mhi, mask, orient, MAX_TIME_DELTA, MIN_TIME_DELTA, 3 );
    
    if( !storage )
        storage = cvCreateMemStorage(0);
    else
        cvClearMemStorage(storage);
    
    // segment motion: get sequence of motion components
    // segmask is marked motion components map. It is not used further
    seq = cvSegmentMotion( mhi, segmask, storage, timestamp, MAX_TIME_DELTA );
    maxcount=0;
    index=0;
    // iterate through the motion components,
    // One more iteration (i == -1) corresponds to the whole image (global motion)
    for( i =0; i < seq->total; i++ ) {

        if( i < 0 ) { // case of the whole image
            comp_rect = cvRect( 0, 0, size.width, size.height );
            color = CV_RGB(255,255,255);
            magnitude1 = 100;
        }
        else { // i-th motion component
            comp_rect = ((CvConnectedComp*)cvGetSeqElem( seq, i ))->rect;
            if( comp_rect.width + comp_rect.height < 150 ) // reject very small components
                continue;
            color = CV_RGB(255,0,0);
            magnitude1 = 30;
        }

        // select component ROI
        cvSetImageROI( silh, comp_rect );
        cvSetImageROI( mhi, comp_rect );
        cvSetImageROI( orient, comp_rect );
        cvSetImageROI( mask, comp_rect );

        count = cvNorm( silh, 0, CV_L1, 0 ); // calculate number of points within silhouette ROI

        cvResetImageROI( mhi );
        cvResetImageROI( orient );
        cvResetImageROI( mask );
        cvResetImageROI( silh );

        // check for the case of little motion
        if( count < comp_rect.width*comp_rect.height * 0.01 )
            continue;
        if (i>=1){
        if (count>maxcount)
         {
           maxcount=count;
           index=i;
          } 
        }
	
	}  
//                 cout<<"img origin="<<img->origin;
//	        cout<<"maxcount="<<maxcount<<"\tindex"<<index<<endl;        
           if(index)
               {  
                comp_rect_temp = ((CvConnectedComp*)cvGetSeqElem( seq, index ))->rect;
                comp_rect_req_center=cvPoint(comp_rect_temp.x+comp_rect_temp.width/2,comp_rect_temp.y+comp_rect_temp.height/2);
                if(comp_rect_req_center.x<=100)
                 {
                   if(comp_rect_req_center.y<=100)
                  comp_rect_req=cvRect(0,0,200,200);
                   else if((comp_rect_req_center.y>100)&&(comp_rect_req_center.y<img->height-100)) 
                       comp_rect_req=cvRect(0,comp_rect_req_center.y-100,200,200);
                   else 
                    comp_rect_req=cvRect(0,img->height-200,200,200);

                 }
                else if(comp_rect_req_center.x>=img->width-100)
                  {
                   if(comp_rect_req_center.y<=100)
                  comp_rect_req=cvRect(img->width-200,0,200,200);
                   else if((comp_rect_req_center.y>100)&&(comp_rect_req_center.y<img->height-100)) 
                       comp_rect_req=cvRect(img->width-200,comp_rect_req_center.y-100,200,200);
                   else 
                    comp_rect_req=cvRect(img->width-200,img->height-200,200,200);
                  }
                else{
                  if(comp_rect_req_center.y<=100)
                  comp_rect_req=cvRect(comp_rect_req_center.x-100,0,200,200);
                   else if((comp_rect_req_center.y>100)&&(comp_rect_req_center.y<img->height-100)) 
                       comp_rect_req=cvRect(comp_rect_req_center.x-100,comp_rect_req_center.y-100,200,200);
                   else 
                    comp_rect_req=cvRect(comp_rect_req_center.x-100,img->height-200,200,200);
                //comp_rect_req=cvRect(comp_rect_req_center.x-100,comp_rect_req_center.y-100,200,200);                 
                 }
       //         cout<<"comp_rect_req_center="<<comp_rect_req_center.x<<"&"<<comp_rect_req_center.y;   
     //    	cout<<"comp_rect_origin="<<comp_rect_req.x<<"&"<<comp_rect_req.y;
                pt1=cvPoint(comp_rect_req.x,comp_rect_req.y);
		pt2=cvPoint(comp_rect_req.x+comp_rect_req.width,comp_rect_req.y);
		pt3=cvPoint(comp_rect_req.x+comp_rect_req.width,comp_rect_req.y+comp_rect_req.height);
		pt4=cvPoint(comp_rect_req.x,comp_rect_req.y+comp_rect_req.height);
	    
		cvLine(motion,pt1,pt2,color,3);
		cvLine(motion,pt2,pt3,color,3);
		cvLine(motion,pt3,pt4,color,3);
		cvLine(motion,pt4,pt1,color,3);
          	
               img_req=cvCreateImage(cvSize(comp_rect_req.width,comp_rect_req.height),8,1);
         
               img_req_rgb=cvCreateImage(cvSize(comp_rect_req.width,comp_rect_req.height),img->depth,3);
         
               cvSetImageROI(img, comp_rect_req); /*cvRect(pt1.x,pt1.y,comp_rect_req.width,comp_rect_req.height));*/

               cvCopy(img,img_req_rgb, NULL);
              
               cvResetImageROI(img);
               cvCvtColor(img_req_rgb,img_req,CV_BGR2GRAY); 
               int height1, width1;
	height1 = img_req->height;
	width1 = img_req->width;
	int nc1 = img_req->nChannels;
	//   uchar* ptr2 =(uchar*) img->imageData;
	int pix_depth1 = img_req->depth;
	CvSize size1;
	size1.width =width1;
	size1.height=height1;
	//cout << "depth1" << pix_depth1 <<  "Channels1" << nc1 << endl;


//	cvNamedWindow("Image required", CV_WINDOW_AUTOSIZE);
//	cvShowImage("Image required", img_req);
	//cvWaitKey();
//	cvDestroyWindow("Image required");
	cvSaveImage("img_req.bmp",img_req);


	int rows1 =(int) height1;
	int cols1 =(int) width1;
	Mat matimg(img_req);

	vector<vector<double> > vec1(rows1, vector<double>(cols1));


	int k =1;
	for (int i=0; i < rows1; i++) {
		for (int j =0; j < cols1; j++){
			unsigned char temp;
			temp = ((uchar*) matimg.data + i * matimg.step)[j  * matimg.elemSize() + k ];
			vec1[i][j] = (double) temp;
		}

	}

	string nm = "db3";
	vector<double> l1,h1,l2,h2;
	filtcoef(nm,l1,h1,l2,h2);
	// unsigned int lf=l1.size();
	//  int rows_n =(int) (rows+ J*(lf-1));
	//  int cols_n =(int)  (cols + J * ( lf -1));

	// Finding 2D DWT Transform of the image using symetric extension algorithm
	// Extension is set to 3 (eg., int e = 3)

	vector<int> length;
	vector<double> output,flag;
	int J =1;
	dwt_2d_sym(vec1,J,nm,output,flag,length);

	double max;
	vector<int> length2;
	// This algorithm computes DWT of image of any given size. Together with convolution and
	// subsampling operations it is clear that subsampled images are of different length than
	// dyadic length images. In order to compute the "effective" size of DWT we do additional
	// calculations.
	dwt_output_dim_sym(length,length2,J);
	// length2 is gives the integer vector that contains the size of subimages that will
	// combine to form the displayed output image. The last two entries of length2 gives the
	// size of DWT ( rows_n by cols_n)

	int siz = length2.size();
	int rows_n=length2[siz-2];
	int cols_n = length2[siz-1];

	vector<vector< double> > dwtdisp(rows_n, vector<double>(cols_n));
	dispDWT(output,dwtdisp, length ,length2, J);

	// dispDWT returns the 2D object dwtdisp which will be displayed using OPENCV's image
	// handling functions

	vector<vector<double> >  dwt_output= dwtdisp;

	maxval(dwt_output,max);// max value is needed to take care of overflow which happens because
	// of convolution operations performed on unsigned 8 bit images

	//Displaying Scaled Image
	// Creating Image in OPENCV
	 // image used for output
	CvSize imgSize; // size of output image

	imgSize.width = cols_n;
	imgSize.height = rows_n;

	cvImg = cvCreateImage( imgSize, 8, 1 );
	// dwt_hold is created to hold the dwt output as further operations need to be
	// carried out on dwt_output in order to display scaled images.
	vector<vector<double> > dwt_hold(rows_n, vector<double>( cols_n));
	dwt_hold = dwt_output;
	// Setting coefficients of created image to the scaled DWT output values
	for (int i = 0; i < imgSize.height; i++ ) {
		for (int j = 0; j < imgSize.width; j++ ){
			if ( dwt_output[i][j] <= 0.0){
				dwt_output[i][j] = 0.0;
			}
			if ( i <= (length2[0]) && j <= (length2[1]) ) {
				((uchar*)(cvImg->imageData + cvImg->widthStep*i))[j] =
						(char) ( (dwt_output[i][j] / max) * 255.0);
			} else {
				((uchar*)(cvImg->imageData + cvImg->widthStep*i))[j] =
						(char) (dwt_output[i][j]) ;
			}
		}
	}

//	cvNamedWindow( "DWT Image", 1 ); // creation of a visualisation window
//	cvShowImage( "DWT Image", cvImg ); // image visualisation
	//cvWaitKey();
//	cvDestroyWindow("DWT Image");
	cvSaveImage("dwt3.bmp",cvImg);

        CvSize lpimgSize; // size of output image

	lpimgSize.width = cvImg->width/2;
	lpimgSize.height = cvImg->height/2;


        img_req_lp=cvCreateImage(lpimgSize,8,1);
        cvSetImageROI(cvImg, cvRect(0, 0,cvImg->width/2, cvImg->height/2));
        //cvCopy(cvImg,img_req_lp, NULL);
        cvResetImageROI(cvImg);
  
  //      cvNamedWindow( "Low_pass", 1 ); // creation of a visualisation window
 //	cvShowImage( "Low_pass", img_req_lp ); // image visualisation
	//cvWaitKey();
//	cvDestroyWindow("Low_pass");
	cvSaveImage("Low_pass3.bmp",img_req_lp);

         
        img_req_hp=cvCreateImage(lpimgSize,8,1);
        cvSetImageROI(cvImg, cvRect( cvImg->width/2,0,cvImg->width,cvImg->height/2 )); 
//      cvSetImageROI(cvImg, cvRect(0, cvImg->height/2,cvImg->width/2,cvImg->height ));
//      cvSetImageROI(cvImg, cvRect(cvImg->width/2, cvImg->height/2,cvImg->width,cvImg->height ));
        cvCopy(cvImg,img_req_hp, NULL);
        cvResetImageROI(cvImg);
  
  //      cvNamedWindow( "High_pass", 1 ); // creation of a visualisation window
//	cvShowImage( "High_pass",img_req_hp ); // image visualisation
	//cvWaitKey();
//	cvDestroyWindow("High_pass");
	cvSaveImage("High_pass3.bmp",img_req_hp);



        double lp_norm,hp_norm ,ratio;
        lp_norm=norm(img_req_lp,NORM_L2);
        hp_norm=norm(img_req_hp,NORM_L2);
        ratio=lp_norm/hp_norm;
        //cout<<"\nlp_norm\t"<<lp_norm<<endl;
        //cout<<"\nhp_norm\t"<<hp_norm<<endl;
        //cout<<"\nratio\t"<<ratio<<endl;
        
        if ((no_samples<5)&&(motor_moving==0))
         {avg_ratio=avg_ratio+ratio;
          no_samples++;
         }
        if (no_samples==5)
         {avg_ratio=avg_ratio/5;
          
          cout <<"avg_ratio="<<avg_ratio<<endl;
           if (avg_ratio >=150)

            {
              motor_moving=1;
              //move motor;
             }
           no_samples=0;
          }    
            
          if ((no_samples_new < 5)&&(motor_moving==1))
               {
                  avg_ratio_new=avg_ratio_new+ratio;    
                  no_samples_new++;  
               }
          if (no_samples_new==5)
               {
                 avg_ratio_new=avg_ratio_new/5;
                 no_samples_new=0; 
                 cout <<"avg_ratio_new="<<avg_ratio_new<<endl;
                  if (avg_ratio_new > avg_ratio)
                   {
                    motor_direction=~(motor_direction);
                    cout<<"\nmotor_direction="<<motor_direction;
                    }
                  if (avg_ratio_new<=80)
                   {
                    motor_moving=0;
                    avg_ratio_new=0;
                    no_samples_new=0;
                    no_samples=0;
                    avg_ratio=0; 
                    }
                   if (avg_ratio_new >80)

                  {//move motor
                
                   cout<<"\nmoving motor"<<endl;
                  }
                   avg_ratio=avg_ratio_new;               
               }
                



              
             
                    



               }
			
//	    cvShowImage( "Motion", motion );

            if( cvWaitKey(10) >= 0 )
                break;
        }
        cvReleaseCapture( &capture );
//        cvDestroyWindow( "Motion" );
    }

    return 0;
}
                                
#ifdef _EiC
main(1,"motempl.c");
#endif

