Description of contents:
./src: contains the code of LGC
./patterngen: used for generating random dot patterns
./examples: two examples using openCV and OSG respectively
./applications: two applications (texture object tracking and engineering drawing augmentation)

Use of the library:
1. Installation of prerequisites (in Ubuntu):
a) Install cmake
b) Install OpenCV (v 3.0.0)
c) Install PCL
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
d) Install OpenSceneGraph

2. Compile all:
run script "compileAll.sh"

3. Run examples and applications
a) example:
Track a point set:
./build/opencvexample -v 0 -r A4Pattern.txt -w 800 -h 600
Augment a point set
./build/osgexample ./build/osgexample.xml

b) applications:
Click and track 2D planar objects (support multi-models):
./build/textureTracking -v 0
Augment different engineering draws (support multi-models):
./build/augmentingDrawings ./models/demo.xml

