# Recall 

How to install:

IGL_LIB = path of the libigl
OPENCV = path of opencv(higher than 3.0)

GetSkeleton.rar is used to generate the skeleton and skin weight file based on the segmentation result.

To input a 3D shape, you would need 5 different files(e.g. files in 'elephant' folder)

You may need to convert the data format from dos/windows to UNIX for the .dmat file to run the program normaly.

In class PoseModels, the function GenerateRecalls is used to generate recalls random and can be called through operation->generate recalls(after load a 3d shape).
The function GenerateSpecificRecall is used to generate specific recall, has no way to call in current version. You can add it to where you need.