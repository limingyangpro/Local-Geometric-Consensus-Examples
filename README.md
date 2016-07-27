# Local-Geometric-Consensus-Examples

## Installation (Linux Only)

### Prerequisites (Unbuntu)
1. Install cmake
sudo apt-get install cmake

2. Install OpenCV (3.1.0)
http://opencv.org/downloads.html

3. Install PCL
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all

4. Install OpenSceneGraph (3.4.0)
http://www.openscenegraph.org/index.php/download-section/stable-releases

5. Download LGC binary

### Complie (Unbuntu)
run script "compileAll.sh"

## Run
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

## Contributing
1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D

## History
9 June 2016: First commit
