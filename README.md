# Local-Geometric-Consensus-Examples

## Executable is available in windows
Download link: <br />
https://www.dropbox.com/s/xf9uraf9u3y7uqf/demo.zip?dl=0

## Installation (Linux Only)

### Prerequisites (Unbuntu)
1. Install cmake<br />
sudo apt-get install cmake

2. Install OpenCV (3.1.0)<br />
http://opencv.org/downloads.html

3. Install PCL<br />
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl<br />
sudo apt-get update<br />
sudo apt-get install libpcl-all<br />

4. Install OpenSceneGraph (3.4.0)<br />
http://www.openscenegraph.org/index.php/download-section/stable-releases

5. Download LGC binary and put inside root/src/<br />
https://www.dropbox.com/s/xmp6sp2n5y1r2z5/liblgc.a?dl=0

### Complie (Unbuntu)
run script "compileAll.sh"

## Run

### Download models and pre-record videos
1. Download models, extract it and put files inside "models" folder:<br />
https://www.dropbox.com/s/7i4hki5kp1qoc9o/models.zip?dl=0

2. Download videos, extract it and put files inside "videos" folder:<br />
https://www.dropbox.com/s/tw7e1e5g4a4pw18/videos.zip?dl=0

### Run examples 
1. cd into "examples" sub-directory<br />
2. run examples:<br />
a) To track a point set:<br />
./build/opencvexample -v ../videos/example.mov -r ../models/A4Pattern.txt -w 1920 -h 1080<br />
b) To augment a point set with a cow:<br />
./build/osgexample ../models/osgexample.xml<br />

### Run applications
1. cd into "applications" sub-directory<br />
2. run applications:<br />
a) To click and track 2D planar objects (support multi-models):<br />
./build/textureTracking -v ../videos/diverseTargets.mpg<br />
b) To augment different engineering draws (support multi-models):<br />
./build/augmentingDrawings ../models/demo.xml<br />

## Contributing
1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D

## History
9 June 2016: First commit
