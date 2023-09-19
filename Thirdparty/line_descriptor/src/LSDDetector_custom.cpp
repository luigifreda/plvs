/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2014, Biagio Montesano, all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#include "precomp_custom.hpp"

//using namespace cv;
namespace cv
{
namespace line_descriptor_c
{

Ptr<LSDDetectorC> LSDDetectorC::createLSDDetectorC(const LSDOptions& parameters)
{
  return Ptr<LSDDetectorC>( new LSDDetectorC(parameters) );
}

/* compute Gaussian pyramid of input image */
void LSDDetectorC::computeGaussianPyramid( const Mat& image, int numOctaves, float scale )
{  
  /* clear class fields */
  vImagePyramid.clear();

  /* insert input image into pyramid */
  cv::Mat currentMat = image.clone();
  //cv::GaussianBlur( currentMat, currentMat, cv::Size( 5, 5 ), 1 ); // N.B.: this was commented also in the original code, no filtering at level 1
  vImagePyramid.push_back( currentMat );

  /* fill Gaussian pyramid */
  for ( int pyrCounter = 1; pyrCounter < numOctaves; pyrCounter++ )
  {
    /* compute and store next image in pyramid and its size */
#if 0    
    // N.B.: this is the original implementation which only supports scale = 2 or integer !
    //std::cout << "scale: "<<  scale << std::endl;
    //std::cout << "src size: " << currentMat.size << ", colsxrows: " << currentMat.cols << " x " << currentMat.rows <<  std::endl; 
    //std::cout << "dsize pyr: " << Size( cvRound((float)currentMat.cols/scale), cvRound((float)currentMat.rows/scale) )  << std::endl;
    pyrDown( currentMat, currentMat, Size( cvRound((float)currentMat.cols/scale), cvRound((float)currentMat.rows/scale) ) );
#else
    cv::GaussianBlur( currentMat, currentMat, cv::Size( 5, 5 ), 1, 1);
    cv::resize( currentMat, currentMat, cv::Size(), ( 1.f / scale ), ( 1.f / scale ), INTER_LINEAR );    
#endif
    
    vImagePyramid.push_back( currentMat );
  }
}

/* check lines' extremes */
inline void checkLineExtremes( cv::Vec4f& extremes, const cv::Size& imageSize, const int borderX=0, const int borderY=0)
{
  const int& minX = borderX;
  const int& minY = borderY;
  const int maxX = (float)imageSize.width  - 1.0f - borderX;
  const int maxY = (float)imageSize.height - 1.0f - borderY;
    
//  if( extremes[0] < 0 )
//    extremes[0] = 0;
  if( extremes[0] < minX )
    extremes[0] = minX;  

//  if( extremes[0] >= imageSize.width )
//    extremes[0] = (float)imageSize.width - 1.0f;
  if( extremes[0] > maxX)
    extremes[0] = maxX;

//  if( extremes[2] < 0 )
//    extremes[2] = 0;
  if( extremes[2] < minX )
    extremes[2] = minX;  

//  if( extremes[2] >= imageSize.width )
//    extremes[2] = (float)imageSize.width - 1.0f;
  if( extremes[2] > maxX )
    extremes[2] = maxX;  

//  if( extremes[1] < 0 )
//    extremes[1] = 0;
  if( extremes[1] < minY )
    extremes[1] = minY;  

//  if( extremes[1] >= imageSize.height )
//    extremes[1] = (float)imageSize.height - 1.0f;
  if( extremes[1] > maxY)
    extremes[1] = maxY;  

//  if( extremes[3] < 0 )
//    extremes[3] = 0;
  if( extremes[3] < minY )
    extremes[3] = minY;  

//  if( extremes[3] >= imageSize.height )
//    extremes[3] = (float)imageSize.height - 1.0f;
  if( extremes[3] > maxY)
    extremes[3] = maxY;  
}

/* requires line detection (only one image) */
void LSDDetectorC::detect( const Mat& image, CV_OUT std::vector<KeyLine>& keylines, float scale, int numOctaves, const Mat& mask )
{
  if( mask.data != NULL && ( mask.size() != image.size() || mask.type() != CV_8UC1 ) )
    throw std::runtime_error( "Mask error while detecting lines: please check its dimensions and that data type is CV_8UC1" );

  else
    detectImpl( image, keylines, numOctaves, scale, mask );
}

/* requires line detection (more than one image) */
void LSDDetectorC::detect( const std::vector<Mat>& images, std::vector<std::vector<KeyLine> >& keylines, float scale, int numOctaves,
                          const std::vector<Mat>& masks ) const
{
  /* detect lines from each image */
  for ( size_t counter = 0; counter < images.size(); counter++ )
  {
    if( masks[counter].data != NULL && ( masks[counter].size() != images[counter].size() || masks[counter].type() != CV_8UC1 ) )
      throw std::runtime_error( "Masks error while detecting lines: please check their dimensions and that data types are CV_8UC1" );

    else
      detectImpl( images[counter], keylines[counter], numOctaves, scale, masks[counter] );
  }
}

/* implementation of line detection */
void LSDDetectorC::detectImpl( const Mat& imageSrc, std::vector<KeyLine>& keylines, int numOctaves, float scale, const Mat& mask ) const
{
    detectImpl(imageSrc, keylines, numOctaves, scale, LSDOptions(), mask );
}

// Overload detect and detectImpl with LSDDetectorC Options
void LSDDetectorC::detect( const Mat& image, CV_OUT std::vector<KeyLine>& keylines, float scale, int numOctaves, const LSDOptions& opts, const Mat& mask )
{
  if( mask.data != NULL && ( mask.size() != image.size() || mask.type() != CV_8UC1 ) )
    throw std::runtime_error( "Mask error while detecting lines: please check its dimensions and that data type is CV_8UC1" );

  else
    detectImpl( image, keylines, numOctaves, scale, opts, mask );
}

void LSDDetectorC::detectImpl( const Mat& imageSrc, std::vector<KeyLine>& keylines, int numOctaves, float scale, const LSDOptions& opts, const Mat& mask ) const
{
  cv::Mat image;
  if( imageSrc.channels() != 1 )
    cvtColor( imageSrc, image, COLOR_BGR2GRAY );
  else
    image = imageSrc.clone();

  /*check whether image depth is different from 0 */
  if( image.depth() != 0 )
    throw std::runtime_error( "Error, depth image!= 0" );

  /* create a pointer to self */
  LSDDetectorC *lsd = const_cast<LSDDetectorC*>( this );

  /* compute Gaussian pyramids */
  //lsd->computeGaussianPyramid( image, numOctaves, scale );
  if(bSetGaussianPyramid)
  {
    // use already set Gaussian pyramid 
    numOctaves = setPyramidNumOctaves;
    scale = setPyramidScale;
    bSetGaussianPyramid = false;
  }
  else
  {
    lsd->computeGaussianPyramid( image, numOctaves, scale );      
  }

  /* create an LSD extractor */
  cv::Ptr<cv::lsd::LineSegmentDetector> ls = cv::lsd::createLineSegmentDetector( opts.refine,
                                                                       opts.scale,
                                                                       opts.sigma_scale,
                                                                       opts.quant,
                                                                       opts.ang_th,
                                                                       opts.log_eps,
                                                                       opts.density_th,
                                                                       opts.n_bins);

  /* prepare a vector to host extracted segments */
  std::vector<std::vector<cv::Vec4f> > lines_lsd;

  /* extract lines */
  for ( int i = 0; i < numOctaves; i++ )
  {
    std::vector<Vec4f> octave_lines;
    ls->detect( vImagePyramid[i], octave_lines );
    lines_lsd.push_back( octave_lines );
  }

  const int diagSize = cvRound( sqrt((float)image.cols*image.cols + (float)image.rows*image.rows ) ); 
  
  /* create keylines */
  int class_counter = -1;
  for ( int octaveIdx = 0; octaveIdx < (int) lines_lsd.size(); octaveIdx++ )
  {
    float octaveScale = std::pow((float)scale, (float)octaveIdx);
    for ( int k = 0; k < (int) lines_lsd[octaveIdx].size(); k++ )
    {
      KeyLine kl;
      cv::Vec4f extremes = lines_lsd[octaveIdx][k];

      /* check data validity */
      checkLineExtremes( extremes, vImagePyramid[octaveIdx].size(), borderX, borderY);

      /* check line segment min length */
      double length = (float) sqrt( pow( extremes[0] - extremes[2], 2 ) + pow( extremes[1] - extremes[3], 2 ) );
      if( length > cvRound( (float)opts.min_length*diagSize/octaveScale ) )
      {
          /* fill KeyLine's fields */
          kl.startPointX = extremes[0] * octaveScale;  
          kl.startPointY = extremes[1] * octaveScale;
          kl.endPointX   = extremes[2] * octaveScale;
          kl.endPointY   = extremes[3] * octaveScale;
          kl.sPointInOctaveX = extremes[0];
          kl.sPointInOctaveY = extremes[1];
          kl.ePointInOctaveX = extremes[2];
          kl.ePointInOctaveY = extremes[3];
          kl.lineLength = length;

          /* compute number of pixels covered by line */
          LineIterator li( vImagePyramid[octaveIdx], Point2f( extremes[0], extremes[1] ), Point2f( extremes[2], extremes[3] ) );
          kl.numOfPixels = li.count;

          kl.angle = atan2( ( kl.endPointY - kl.startPointY ), ( kl.endPointX - kl.startPointX ) );
          kl.class_id = ++class_counter;
          kl.octave = octaveIdx;
          kl.size = ( kl.endPointX - kl.startPointX ) * ( kl.endPointY - kl.startPointY );
          kl.response = kl.lineLength / max( vImagePyramid[octaveIdx].cols, vImagePyramid[octaveIdx].rows );
          kl.pt = Point2f( ( kl.endPointX + kl.startPointX ) / 2, ( kl.endPointY + kl.startPointY ) / 2 );

          keylines.push_back( kl );
      }
    }
  }

  /* delete undesired KeyLines, according to input mask */
  if( !mask.empty() )
  {
    //for ( size_t keyCounter = 0; keyCounter < keylines.size(); keyCounter++ )
    for ( std::vector<KeyLine>::iterator it = keylines.begin(); it != keylines.end();  )  
    {
      //KeyLine& kl = keylines[keyCounter];
      KeyLine& kl = *it;      
      
//      if( mask.at<uchar>( (int) kl.startPointY, (int) kl.startPointX ) == 0 && mask.at<uchar>( (int) kl.endPointY, (int) kl.endPointX ) == 0 )
//      {
//        keylines.erase( keylines.begin() + keyCounter );
//        keyCounter--;
//      }
      if( mask.at < uchar > ( (int) kl.startPointY, (int) kl.startPointX ) == 0 && mask.at < uchar > ( (int) kl.endPointY, (int) kl.endPointX ) == 0 )
      {
        it = keylines.erase(it);
      }
      else
      {
        ++it;
      }      
    }
  }

}


/* implementation of line detection */
void LSDDetectorC::detectImplOld( const Mat& imageSrc, std::vector<KeyLine>& keylines, int numOctaves, float scale, const Mat& mask ) const
{    
  cv::Mat image;
  if( imageSrc.channels() != 1 )
    cvtColor( imageSrc, image, COLOR_BGR2GRAY );
  else
    image = imageSrc.clone();

  /*check whether image depth is different from 0 */
  if( image.depth() != 0 )
    throw std::runtime_error( "Error, depth image!= 0" );

  /* create a pointer to self */
  LSDDetectorC *lsd = const_cast<LSDDetectorC*>( this );

  /* compute Gaussian pyramids */
  lsd->computeGaussianPyramid( image, numOctaves, scale );

  /* create an LSD extractor */
  cv::Ptr<cv::lsd::LineSegmentDetector> ls = cv::lsd::createLineSegmentDetector( cv::LSD_REFINE_ADV );

  /* prepare a vector to host extracted segments */
  std::vector<std::vector<cv::Vec4f> > lines_lsd;

  /* extract lines */
  for ( int i = 0; i < numOctaves; i++ )
  {
    std::vector<Vec4f> octave_lines;
    ls->detect( vImagePyramid[i], octave_lines );
    lines_lsd.push_back( octave_lines );
  }

  /* create keylines */
  int class_counter = -1;
  for ( int octaveIdx = 0; octaveIdx < (int) lines_lsd.size(); octaveIdx++ )
  {
    float octaveScale = std::pow((float)scale, (float)octaveIdx);
    for ( int k = 0; k < (int) lines_lsd[octaveIdx].size(); k++ )
    {
      KeyLine kl;
      cv::Vec4f extremes = lines_lsd[octaveIdx][k];

      /* check data validity */
      checkLineExtremes( extremes, vImagePyramid[octaveIdx].size(), borderX, borderY);

      /* fill KeyLine's fields */
      kl.startPointX = extremes[0] * octaveScale;
      kl.startPointY = extremes[1] * octaveScale;
      kl.endPointX = extremes[2] * octaveScale;
      kl.endPointY = extremes[3] * octaveScale;
      kl.sPointInOctaveX = extremes[0];
      kl.sPointInOctaveY = extremes[1];
      kl.ePointInOctaveX = extremes[2];
      kl.ePointInOctaveY = extremes[3];
      kl.lineLength = (float) sqrt( pow( extremes[0] - extremes[2], 2 ) + pow( extremes[1] - extremes[3], 2 ) );

      /* compute number of pixels covered by line */
      LineIterator li( vImagePyramid[octaveIdx], Point2f( extremes[0], extremes[1] ), Point2f( extremes[2], extremes[3] ) );
      kl.numOfPixels = li.count;

      kl.angle = atan2( ( kl.endPointY - kl.startPointY ), ( kl.endPointX - kl.startPointX ) );
      kl.class_id = ++class_counter;
      kl.octave = octaveIdx;
      kl.size = ( kl.endPointX - kl.startPointX ) * ( kl.endPointY - kl.startPointY );
      kl.response = kl.lineLength / max( vImagePyramid[octaveIdx].cols, vImagePyramid[octaveIdx].rows );
      kl.pt = Point2f( ( kl.endPointX + kl.startPointX ) / 2, ( kl.endPointY + kl.startPointY ) / 2 );

      keylines.push_back( kl );
    }
  }

  /* delete undesired KeyLines, according to input mask */
  if( !mask.empty() )
  {
    for ( size_t keyCounter = 0; keyCounter < keylines.size(); keyCounter++ )
    {
      KeyLine kl = keylines[keyCounter];
      if( mask.at<uchar>( (int) kl.startPointY, (int) kl.startPointX ) == 0 && mask.at<uchar>( (int) kl.endPointY, (int) kl.endPointX ) == 0 )
      {
        keylines.erase( keylines.begin() + keyCounter );
        keyCounter--;
      }
    }
  }

}


void LSDDetectorC::setGaussianPyramid(const std::vector<cv::Mat>& pyrs, int numOctaves, float scale, int borderX_in, int borderY_in)
{
    assert(borderX_in>=0 && borderY_in>=0);
    borderX = borderX_in;
    borderY = borderY_in;
    
    bSetGaussianPyramid = true; 
    
    setPyramidNumOctaves = std::min(numOctaves, std::min(options.numOctaves, (int)pyrs.size()) );
    setPyramidScale = scale; 
    
    if ((borderX > 0) || (borderY > 0))
    {
        vImagePyramid.resize(setPyramidNumOctaves);       
        
        for (int i = 0; i < numOctaves; i++)
        {
            const int rows = pyrs[i].rows - 1 - borderY;
            const int cols = pyrs[i].cols - 1 - borderX;
            if ((rows > 0) && (cols > 0))
            {
                // detect lines by considering fixed border at all octave 
                vImagePyramid[i] = pyrs[i].rowRange(borderY, rows).colRange(borderX, cols);
            }
            else
            {
                throw std::runtime_error("Error, rows and cols are not > 0");
            }
        }
    }
    else
    {
        vImagePyramid = pyrs; // implicit memory sharing when copying of element images 
    }
}



}

}

