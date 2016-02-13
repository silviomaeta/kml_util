# kml_util
Generate KML file from ROS Odometry data. Enable visualization of traversed path in GoogleEarth.

Author: Silvio Maeta - smaeta@andrew.cmu.edu

Install required packages:
    $ apt-get update
    $ apt-get install python-pip
    $ pip install utm
    $ easy-install pykml

Take a look in the kml_util/launch/generate_kml.launch to configure:
 - output_filename: output path and filename for the PDF report containing the plots
 - pose_topic: Odometry message topic name - position information needs to be in UTM coordinate format. 
 - utm_zone_number: UTM zone number is required because UTM coordinates are converted to LatLong in order to be saved in the KML file.
 
 OBS: Odometry position expected in NED coordinate frame (North=X / East=Y / Down=Z)
 
