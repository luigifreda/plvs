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

namespace cv
{
namespace line_descriptor_c
{
/* draw matches between two images */
void drawLineMatches( const Mat& img1, const std::vector<KeyLine>& keylines1, const Mat& img2, const std::vector<KeyLine>& keylines2,
                      const std::vector<DMatch>& matches1to2, Mat& outImg, const Scalar& matchColor, const Scalar& singleLineColor,
                      const std::vector<char>& matchesMask, int flags )
{

  if(img1.type() != img2.type())
  {
    std::cout << "Input images have different types" << std::endl;
    CV_Assert(img1.type() == img2.type());
  }

  /* initialize output matrix (if necessary) */
  if( flags == DrawLinesMatchesFlags::DEFAULT )
  {
    /* check how many rows are necessary for output matrix */
    int totalRows = img1.rows >= img2.rows ? img1.rows : img2.rows;

    /* initialize output matrix */
    outImg = Mat::zeros( totalRows, img1.cols + img2.cols, img1.type() );

  }

  /* initialize random seed: */
  srand( (unsigned int) time( NULL ) );

  Scalar singleLineColorRGB;
  if( singleLineColor == Scalar::all( -1 ) )
  {
    int R = ( rand() % (int) ( 255 + 1 ) );
    int G = ( rand() % (int) ( 255 + 1 ) );
    int B = ( rand() % (int) ( 255 + 1 ) );

    singleLineColorRGB = Scalar( R, G, B );
  }

  else
    singleLineColorRGB = singleLineColor;

  /* copy input images to output images */
  Mat roi_left( outImg, Rect( 0, 0, img1.cols, img1.rows ) );
  Mat roi_right( outImg, Rect( img1.cols, 0, img2.cols, img2.rows ) );
  img1.copyTo( roi_left );
  img2.copyTo( roi_right );

  /* get columns offset */
  int offset = img1.cols;

  /* if requested, draw lines from both images */
  if( flags != DrawLinesMatchesFlags::NOT_DRAW_SINGLE_LINES )
  {
    for ( size_t i = 0; i < keylines1.size(); i++ )
    {
      KeyLine k1 = keylines1[i];
      line( outImg, Point2f( k1.startPointX, k1.startPointY ), Point2f( k1.endPointX, k1.endPointY ), singleLineColorRGB, 2 );
      //line( outImg, Point2f( k1.sPointInOctaveX, k1.sPointInOctaveY ), Point2f( k1.ePointInOctaveX, k1.ePointInOctaveY ), singleLineColorRGB, 2 );

    }

    for ( size_t j = 0; j < keylines2.size(); j++ )
    {
      KeyLine k2 = keylines2[j];
      //line( outImg, Point2f( k2.sPointInOctaveX + offset, k2.sPointInOctaveY ), Point2f( k2.ePointInOctaveX + offset, k2.ePointInOctaveY ), singleLineColorRGB, 2 );
      line( outImg, Point2f( k2.startPointX + offset, k2.startPointY ), Point2f( k2.endPointX + offset, k2.endPointY ), singleLineColorRGB, 2 );
    }
  }

  /* draw matches */
  for ( size_t counter = 0; counter < matches1to2.size(); counter++ )
  {
    if( matchesMask[counter] != 0 )
    {
      const DMatch& dm = matches1to2[counter];
      const KeyLine& left = keylines1[dm.queryIdx];
      const KeyLine& right = keylines2[dm.trainIdx];

      Scalar matchColorRGB;
      if( matchColor == Scalar::all( -1 ) )
      {
        int R = ( rand() % (int) ( 255 + 1 ) );
        int G = ( rand() % (int) ( 255 + 1 ) );
        int B = ( rand() % (int) ( 255 + 1 ) );

        matchColorRGB = Scalar( R, G, B );

        if( singleLineColor == Scalar::all( -1 ) )
          singleLineColorRGB = matchColorRGB;
      }

      else
        matchColorRGB = matchColor;

      /* draw lines if necessary */
      line( outImg, Point2f( left.startPointX, left.startPointY ), Point2f( left.endPointX, left.endPointY ), singleLineColorRGB, 2 );

      line( outImg, Point2f( right.startPointX + offset, right.startPointY ), Point2f( right.endPointX + offset, right.endPointY ), singleLineColorRGB,
            2 );

      /* link correspondent lines */
      line( outImg, Point2f( left.startPointX, left.startPointY ), Point2f( right.startPointX + offset, right.startPointY ), matchColorRGB, 1 );

//      
//        line( outImg, Point2f( left.sPointInOctaveX, left.sPointInOctaveY ), Point2f( left.ePointInOctaveX, left.ePointInOctaveY ), singleLineColorRGB, 2 );
//
//        line( outImg, Point2f( right.sPointInOctaveX + offset, right.sPointInOctaveY ), Point2f( right.ePointInOctaveX + offset, right.ePointInOctaveY ), singleLineColorRGB,
//              2 );
//
//        // link correspondent lines 
//        line( outImg, Point2f( left.sPointInOctaveX, left.sPointInOctaveY ), Point2f( right.sPointInOctaveX + offset, right.sPointInOctaveY ), matchColorRGB, 1 );
//       
    }
  }
}

void drawLineMatchesStereo( const Mat& img1L, const Mat& img1R, const std::vector<KeyLine>& keylines1L, const std::vector<KeyLine>& keylines1R, int N1Left,
                            const Mat& img2L, const Mat& img2R, const std::vector<KeyLine>& keylines2L, const std::vector<KeyLine>& keylines2R, int N2Left,
                            const std::vector<DMatch>& matches1to2, Mat& outImg, const Scalar& matchColor,
                            const Scalar& singleLineColor, const std::vector<char>& matchesMask,
                            int flags) 
{

  if(img1L.type() != img1R.type() || img2L.type() != img2R.type() || img1L.type() != img2L.type())
  {
    std::cout << "Input images have different types" << std::endl;
    CV_Assert(img1L.type() == img1R.type() && img1L.type() == img2L.type() && img1L.type() == img2R.type());
  }

  /* initialize output matrix (if necessary) */
  if( flags == DrawLinesMatchesFlags::DEFAULT )
  {
    CV_Assert(img1L.rows == img2L.rows && img1L.cols == img2L.cols && 
              img1R.rows == img2R.rows && img1R.cols == img2R.cols && 
              img1L.rows == img1R.rows && img1L.cols == img1R.cols);
    /* check how many rows are necessary for output matrix */
    int totalRows = img1L.rows + img2L.rows;
    int totalCols = img1L.cols + img1R.cols;

    /* initialize output matrix */
    outImg = Mat::zeros( totalRows, totalCols, img1L.type() );

  }

  /* initialize random seed: */
  srand( (unsigned int) time( NULL ) );

  Scalar singleLineColorRGB;
  if( singleLineColor == Scalar::all( -1 ) )
  {
    int R = ( rand() % (int) ( 255 + 1 ) );
    int G = ( rand() % (int) ( 255 + 1 ) );
    int B = ( rand() % (int) ( 255 + 1 ) );

    singleLineColorRGB = Scalar( R, G, B );
  }

  else
    singleLineColorRGB = singleLineColor;

  /* copy input images to output images */
  Mat roi1_left( outImg, Rect( 0, 0, img1L.cols, img1L.rows ) );
  Mat roi1_right( outImg, Rect( img1L.cols, 0, img1R.cols, img1R.rows ) );
  img1L.copyTo( roi1_left );
  img1R.copyTo( roi1_right );

  Mat roi2_left( outImg, Rect( 0, img1L.rows, img2L.cols, img2L.rows ) );
  Mat roi2_right( outImg, Rect( img1L.cols, img1L.rows, img2R.cols, img2R.rows ) );
  img2L.copyTo( roi2_left );
  img2R.copyTo( roi2_right );  

  /* get columns offset */
  const int offsetx = img1L.cols;
  const int offsety = img1L.rows;

  /* if requested, draw lines from both images */
  if( flags != DrawLinesMatchesFlags::NOT_DRAW_SINGLE_LINES )
  {
    for ( size_t i = 0; i < keylines1L.size(); i++ )
    {
      const KeyLine& k1 = keylines1L[i];
      line( outImg, Point2f( k1.startPointX, k1.startPointY ), Point2f( k1.endPointX, k1.endPointY ), singleLineColorRGB, 2 );
      //line( outImg, Point2f( k1.sPointInOctaveX, k1.sPointInOctaveY ), Point2f( k1.ePointInOctaveX, k1.ePointInOctaveY ), singleLineColorRGB, 2 );

    }

    for ( size_t i = 0; i < keylines1R.size(); i++ )
    {
      const KeyLine& k1 = keylines1R[i];
      line( outImg, Point2f( k1.startPointX+offsetx, k1.startPointY ), Point2f( k1.endPointX+offsetx, k1.endPointY ), singleLineColorRGB, 2 );
      //line( outImg, Point2f( k1.sPointInOctaveX, k1.sPointInOctaveY ), Point2f( k1.ePointInOctaveX, k1.ePointInOctaveY ), singleLineColorRGB, 2 );

    }

    for ( size_t j = 0; j < keylines2L.size(); j++ )
    {
      KeyLine k2 = keylines2L[j];
      //line( outImg, Point2f( k2.sPointInOctaveX + offset, k2.sPointInOctaveY ), Point2f( k2.ePointInOctaveX + offset, k2.ePointInOctaveY ), singleLineColorRGB, 2 );
      line( outImg, Point2f( k2.startPointX, k2.startPointY + offsety ), Point2f( k2.endPointX, k2.endPointY + offsety ), singleLineColorRGB, 2 );
    }

    for ( size_t j = 0; j < keylines2R.size(); j++ )
    {
      KeyLine k2 = keylines2R[j];
      //line( outImg, Point2f( k2.sPointInOctaveX + offset, k2.sPointInOctaveY ), Point2f( k2.ePointInOctaveX + offset, k2.ePointInOctaveY ), singleLineColorRGB, 2 );
      line( outImg, Point2f( k2.startPointX + offsetx, k2.startPointY + offsety ), Point2f( k2.endPointX + offsetx, k2.endPointY + offsety ), singleLineColorRGB, 2 );
    }    
  }

  /* draw matches */
  for ( size_t counter = 0; counter < matches1to2.size(); counter++ )
  {
    if( matchesMask[counter] != 0 )
    {
      const DMatch& dm = matches1to2[counter];
      const auto& idx1 = dm.queryIdx;
      const auto& idx2 = dm.trainIdx;

      const KeyLine& l1 = (idx1 < N1Left) ? keylines1L[idx1] : keylines1R[idx1 - N1Left];
      const KeyLine& l2 = (idx2 < N2Left) ? keylines2L[idx2] : keylines2R[idx2 - N2Left];

      const bool bRight1 = idx1 < N1Left ? false : true;
      const bool bRight2 = idx2 < N2Left ? false : true;       

      const int offset1x = bRight1 ? offsetx : 0;
      const int offset1y = 0; 
      
      const int offset2x = bRight2 ? offsetx : 0;
      const int offset2y = offsety;

      Scalar matchColorRGB;
      if( matchColor == Scalar::all( -1 ) )
      {
        int R = ( rand() % (int) ( 255 + 1 ) );
        int G = ( rand() % (int) ( 255 + 1 ) );
        int B = ( rand() % (int) ( 255 + 1 ) );

        matchColorRGB = Scalar( R, G, B );

        if( singleLineColor == Scalar::all( -1 ) )
          singleLineColorRGB = matchColorRGB;
      }

      else
        matchColorRGB = matchColor;

      /* draw lines if necessary */
      line( outImg, Point2f( l1.startPointX + offset1x, l1.startPointY ), Point2f( l1.endPointX + offset1x, l1.endPointY ), singleLineColorRGB, 2 );

      line( outImg, Point2f( l2.startPointX + offset2x, l2.startPointY + offset2y), Point2f( l2.endPointX + offset2x, l2.endPointY + offset2y), singleLineColorRGB, 2);

      /* link correspondent lines */
      line( outImg, Point2f( l1.startPointX + offset1x, l1.startPointY ), Point2f( l2.startPointX + offset2x, l2.startPointY + offset2y), matchColorRGB, 1 ); 
    }
  }  
}

/* draw extracted lines on original image */
void drawKeylines( const Mat& image, const std::vector<KeyLine>& keylines, Mat& outImage, const Scalar& color, int flags )
{
  if( flags == DrawLinesMatchesFlags::DEFAULT )
    outImage = image.clone();

  for ( size_t i = 0; i < keylines.size(); i++ )
  {
    /* decide lines' color  */
    Scalar lineColor;
    if( color == Scalar::all( -1 ) )
    {
      int R = ( rand() % (int) ( 255 + 1 ) );
      int G = ( rand() % (int) ( 255 + 1 ) );
      int B = ( rand() % (int) ( 255 + 1 ) );

      lineColor = Scalar( R, G, B );
    }

    else
      lineColor = color;

    /* get line */
    KeyLine k = keylines[i];

    /* draw line */
    line( outImage, Point2f( k.startPointX, k.startPointY ), Point2f( k.endPointX, k.endPointY ), lineColor, 1 );
  }
}

}
}
