#!/usr/bin/env python
'''
GPS viewer
'''
# Woensug choi, woensug.choi.ks@nps.edu
# Referenced from https://stackoverflow.com/a/68615108/17983250

# ----- Requirements ----- #
# Install required python3 modules
# pip install google-api-python-client earthengine-api pyCrypto
# pip install earthengine-api --upgrade
# pip install folium

# Authenticate google earth engine
# earthengine authenticate
# ------------------------ #

# Import system modulees
import sys, signal, io

# ROS/Gazebo imports
import rospy
from nav_msgs.msg import Odometry

# For lat/lon coordinate frame transformations
from osgeo import ogr
from osgeo import osr

# Imports for Maps
import folium
from folium import plugins

# Imports for GUI
from jinja2 import Template
from PyQt5.QtCore import pyqtSignal, QObject, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5.QtWebEngineWidgets import QWebEngineView

# Global variables
refresh_rate = 2 # in seconds
default_zoom = 10
save_html = True
robot_name = 'rexrov'
topic_name = '/rexrov/pose_gt'
latitude = 0.0
longitude = 0.0
current_time = 0.0

###############################################
class CoordinateProvider(QObject):
    coordinate_changed = pyqtSignal(float, float)

    def __init__(self, parent=None):
        global refresh_rate
        # get variables from parent class
        super().__init__(parent)

        # Timer for GUI
        self._timer = QTimer(interval=int(1000*refresh_rate))
        self._timer.timeout.connect(self.generate_coordinate)

    def start(self):
        self._timer.start()

    def stop(self):
        self._timer.stop()

    def generate_coordinate(self):
        global latitude, longitude
        self.coordinate_changed.emit(float(latitude), float(longitude))


###############################################
class Window(QMainWindow):

    def __init__(self):

        global latitude, longitude, default_zoom
        super().__init__()

        # Parameters
        coordinate = (latitude, longitude)

        # Add Earth Engine drawing method to folium.
        self.map = folium.Map(location=coordinate, tiles="Stamen Toner", zoom_start=default_zoom)

        # GMRT WMS (https://www.gmrt.org/services/index.php)
        folium.raster_layers.WmsTileLayer(
          url='http://www.gmrt.org/services/mapserver/wms_merc?request=GetCapabilities&service=WMS&version=1.3.0', \
          layers='GMRT', version='1.3.0', fmt='image/png',
          name='GMRT', attr=u'GMRT',
          overlay=True, control=True, pixelated=True, opacity=0.6,
        ).add_to(self.map)

        # Add layer control panel
        folium.LayerControl().add_to(self.map)

        # GPS
        plugins.LocateControl().add_to(self.map)

        # Mouse position
        fmtr = "function(num) {return L.Util.formatNum(num, 3) + ' º ';};"
        plugins.MousePosition(position='topright', separator=' | ', prefix="Mouse:",\
                    lat_formatter=fmtr, lng_formatter=fmtr).add_to(self.map)

        # Add the draw
        plugins.Draw(export=True, filename='data.geojson', position='topleft', \
                    draw_options=None, edit_options=None).add_to(self.map)

        # Add measure tool
        plugins.MeasureControl(position='topright', primary_length_unit='meters', \
                    secondary_length_unit='miles', primary_area_unit='sqmeters', \
                    secondary_area_unit='acres').add_to(self.map)

        # Add initial marker
        folium.Marker(coordinate).add_to(self.map)

        data = io.BytesIO()
        self.map.save(data, close_file=False)

        self.map_view = QWebEngineView()
        self.map_view.setHtml(data.getvalue().decode())

        self.setCentralWidget(self.map_view)

    def add_marker(self):

        global latitude, longitude, robot_name, current_time

        # For logging
        folium.Marker(location=[latitude, longitude],
          tooltip='<b>' + robot_name + '<br>Time:</b> ' + repr(current_time) + ' s' \
                +'<br><b>Lat</b>: ' + repr(latitude) + '<br><b>Lon</b>: ' + repr(longitude),
          popup='<b>' + robot_name + '<br>Time:</b> ' + repr(current_time) + ' s' \
                +'<br><b>Lat</b>: ' + repr(latitude) + '<br><b>Lon</b>: ' + repr(longitude) \
          ).add_to(self.map)

        # For live marker
        # leafletjs format being rendered with Flask/Jinja2
        js = Template(
            """
        L.marker([{{latitude}}, {{longitude}}],
            ).addTo({{map}})
             .bindPopup('<b>Time:</b> {{current_time|float}} s <br>'
              + '<b>Lat:</b> {{'%0.5f' % latitude|float }}<br>'
              + '<b>Lon:</b> {{'%0.5f' % longitude|float }}')
             .openPopup();
        """
        ).render(map=self.map.get_name(), \
          latitude=latitude, longitude=longitude, current_time=current_time)
        self.map_view.page().runJavaScript(js)

        # Save as html
        self.map.save('/tmp/GPSViewer_log.html')

###############################################
def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    if QMessageBox.question(None, '', "Are you sure you want to quit?",
                            QMessageBox.Yes | QMessageBox.No,
                            QMessageBox.No) == QMessageBox.Yes:
        QApplication.quit()


###############################################
def reproject(x, y):

    global latitude, longitude

    # Calculate center coordinates from X/Y (UTM; epsg3857)
    source = osr.SpatialReference()
    source.ImportFromEPSG(3857)
    target = osr.SpatialReference()
    target.ImportFromEPSG(4326)
    transform = osr.CoordinateTransformation(source, target)
    point = ogr.CreateGeometryFromWkt("POINT (" + \
          repr(x) + " " + \
          repr(y) + ")"
          )
    point.Transform(transform)
    latitude = point.ExportToWkt().split(" ")[1].split("(")[1]
    longitude = point.ExportToWkt().split(" ")[2].split(")")[0]

###############################################
def StartGUI():

    global topic_name, latitude, longitude

    # Pre-recognize Ctrl+C
    # signal.signal(signal.SIGINT, sigint_handler)

    # Start QtApp
    app = QApplication(sys.argv)

    # Read initial lat/lon
    init_msg = rospy.wait_for_message(topic_name, Odometry)
    reproject(init_msg.pose.pose.position.x, init_msg.pose.pose.position.y)

    # Open GUI window
    window = Window()
    window.show()
    # window.showMaximized()

    # Update marker
    provider = CoordinateProvider()
    provider.coordinate_changed.connect(window.add_marker)
    provider.start()

    # Recognize Ctrl+C
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec())


###############################################
class GetModelPose:

    def __init__(self):

        global refresh_rate, save_html, default_zoom, \
               topic_name, robot_name, latitude, longitude, \
               current_time

        robot_name = rospy.get_param('~namespace', 'rexrov')
        topic_name = '/'+ robot_name + '/pose_gt'
        refresh_rate = rospy.get_param('~refresh_rate', 2.0)
        save_html = rospy.get_param('~save_html', True)
        default_zoom = rospy.get_param('~default_zoom', 11)

        # Wait for ROS init
        self.t0 = rospy.get_time()
        while self.t0 < refresh_rate:
            rospy.logwarn("Waiting for ROS time to be received")
            rospy.sleep(0.5)
            self.t0 = rospy.get_time()

        # Initiate QT GUI Window
        rospy.loginfo("Launching GPS Viewer with lat/lon from " + topic_name)
        rospy.Subscriber(topic_name, Odometry, self.get_model_position)

    def get_model_position(self, data):
        global current_time
        reproject(data.pose.pose.position.x, data.pose.pose.position.y)
        current_time = data.header.stamp.secs

if __name__ == "__main__":

    # Start node
    rospy.init_node('gps_viewer', anonymous=True)
    node = GetModelPose()
    StartGUI()

    rospy.spin()
