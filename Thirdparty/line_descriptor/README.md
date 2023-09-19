Binary descriptors for lines extracted from an image
====================================================

This is part of PLVS. Some bugs in the original code have been fixed. 

This module shows how to extract line segments from an image by 2 different methods: 
1) segmenting lines with Line Segment Detector LSDDetector 
2) using the Binary Descriptor to get the lines and give them a descriptor -- BinaryDescriptor. 
Finally, we can then match line segments using the BinaryDescriptorMatcher class.
