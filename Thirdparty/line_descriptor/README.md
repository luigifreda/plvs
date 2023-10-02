Binary descriptors for lines extracted from an image
====================================================

This is a modifed version of the [line_descriptor](https://github.com/opencv/opencv_contrib/tree/4.x/modules/line_descriptor) module which is part of [OpenCV contrib](https://github.com/opencv/opencv_contrib/tree/4.x) repository. This is part of the PLVS framework. Some bugs in the original code were fixed. Plus, some parts were improved and optimized.  

This module shows how to extract line segments from an image by 2 different methods: 
1) segmenting lines with Line Segment Detector LSDDetector 
2) using the Binary Descriptor to get the lines and give them a descriptor -- BinaryDescriptor. 
Finally, we can then match line segments using the BinaryDescriptorMatcher class.
