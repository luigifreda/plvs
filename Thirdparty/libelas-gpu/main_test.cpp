#include <cmath>
#include <iostream>
#include "image.h"

using namespace std;
using namespace libelas;

// compute disparities of pgm image input pair file_1, file_2
void process (const char* file_1,const char* file_2) {

  cout << "Processing: " << file_1 << ", " << file_2 << endl;

  // load images
  image<uchar> *I1,*I2;
  I1 = loadPGM(file_1);
  I2 = loadPGM(file_2);

  // check for correct size
  if (I1->width()<=0 || I1->height() <=0 || I2->width()<=0 || I2->height() <=0 ||
      I1->width()!=I2->width() || I1->height()!=I2->height()) {
    cout << "ERROR: Images must be of same size, but" << endl;
    cout << "       I1: " << I1->width() <<  " x " << I1->height() << 
                 ", I2: " << I2->width() <<  " x " << I2->height() << endl;
    delete I1;
    delete I2;
    return;
  }

  // Get image width and height
  int32_t width  = I1->width();
  int32_t height = I1->height();

  // Allocate the calculation image matrix
  //image<uchar> *I3(width, height, true);

  // Variable for the total error
  double sse = 0;

  // Compute the mean squared error between the two images
  // http://stackoverflow.com/a/17237076
  for(int32_t i=0; i<width; i++) {
    for(int32_t j=0; j<height; j++) {
      // cout << index << " - " <<  << endl;
      sse += pow(abs(imRef(I1, i, j) - imRef(I2, i, j)),2);
    }
  }

  // MSE = sum((frame1-frame2)^2 ) / no. of pixels
  double mse = sse/ (double)(width*height);

  // Print it
  cout << "Mean Squared Error (MSE) = " << mse << endl;
  
  // free memory
  delete I1;
  delete I2;
}

int main (int argc, char** argv) {

  // Run the comparison between the two images
  if (argc==2 && !strcmp(argv[1],"demo")) {
    // Images (hard coded for dev'ing)
    process("../GPU_test/2016_12_06_cpu/cones_left_disp.pgm",   "../GPU_test/2016_12_06_gpu/cones_left_disp.pgm");
    process("../GPU_test/2016_12_06_cpu/cones_right_disp.pgm",   "../GPU_test/2016_12_06_gpu/cones_right_disp.pgm");
    process("../GPU_test/2016_12_06_cpu/aloe_left_disp.pgm",    "../GPU_test/2016_12_06_gpu/aloe_left_disp.pgm");
    process("../GPU_test/2016_12_06_cpu/aloe_right_disp.pgm",    "../GPU_test/2016_12_06_gpu/aloe_right_disp.pgm");
    process("../GPU_test/2016_12_06_cpu/raindeer_left_disp.pgm","../GPU_test/2016_12_06_gpu/raindeer_left_disp.pgm");
    process("../GPU_test/2016_12_06_cpu/raindeer_right_disp.pgm","../GPU_test/2016_12_06_gpu/raindeer_right_disp.pgm");
    process("../GPU_test/2016_12_06_cpu/urban1_left_disp.pgm",  "../GPU_test/2016_12_06_gpu/urban1_left_disp.pgm");
    process("../GPU_test/2016_12_06_cpu/urban1_right_disp.pgm",  "../GPU_test/2016_12_06_gpu/urban1_right_disp.pgm");
    process("../GPU_test/2016_12_06_cpu/urban2_left_disp.pgm",  "../GPU_test/2016_12_06_gpu/urban2_left_disp.pgm");
    process("../GPU_test/2016_12_06_cpu/urban2_right_disp.pgm",  "../GPU_test/2016_12_06_gpu/urban2_right_disp.pgm");
    process("../GPU_test/2016_12_06_cpu/urban3_left_disp.pgm",  "../GPU_test/2016_12_06_gpu/urban3_left_disp.pgm");
    process("../GPU_test/2016_12_06_cpu/urban3_right_disp.pgm",  "../GPU_test/2016_12_06_gpu/urban3_right_disp.pgm");
    process("../GPU_test/2016_12_06_cpu/urban4_left_disp.pgm",  "../GPU_test/2016_12_06_gpu/urban4_left_disp.pgm");
    process("../GPU_test/2016_12_06_cpu/urban4_right_disp.pgm",  "../GPU_test/2016_12_06_gpu/urban4_right_disp.pgm");
    cout << "... done!" << endl;

  // compute disparity from input pair
  } else if (argc==3) {
    process(argv[1],argv[2]);
    cout << "... done!" << endl;

  } else {
    cerr << "Please specify the two images you want to compare" << endl;
    cerr << "./main_test <path_cpu_img> <path_gpu_img>" << endl;
  }

  return 0;
}


