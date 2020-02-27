/************************************************************/
/*    NAME: David Battle                                    */
/*    ORGN: MISSION SYSTEMS PTY LTD                         */
/*    FILE: GridViewer.cpp                                  */
/*    DATE: 17 Dec 2018                                     */
/************************************************************/
#include "GridViewer.h"
#include "MBUtils.h"
#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;

//---------------------------------------------------------
// Constructor

GridViewer::GridViewer()
{
  m_iterations = 0;
  m_timewarp   = 1;
  m_lidar_frame = false;
  m_point_size = 1;
  m_resolution = 1.0f;
  m_display_depth = 0;
  m_down_count = 0;
  m_up_count = 0;
  m_auto_level = true;

  // Camera params
  m_camera = "Camera unset";
  m_camera_update = false;
  m_fov_deg = 90.0;
  m_focus_x = 0.f;
  m_focus_y = 0.f;
  m_focus_z = 0.f;
  m_pos_x = 0.f;
  m_pos_y = 0.f;
  m_pos_z = 0.f;
  m_up_x = 0.f;
  m_up_y = 0.f;
  m_up_z = 1.f;

  // Instantiate PCL Visualizer object
  m_viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>
      (new pcl::visualization::PCLVisualizer ("pGridViewer"));

  m_viewer->setBackgroundColor(0, 0, 0);
  m_viewer->addCoordinateSystem(1.0);
  m_viewer->initCameraParameters();

  // Instantiate Octree object
  m_octree = boost::shared_ptr<pcl::octree::OctreePointCloudOccupancy<RGBA_point>>
      (new pcl::octree::OctreePointCloudOccupancy<RGBA_point> (1.0f));

  // Instantiate display cloud object
  m_displayCloud = RGBA_cloud_ptr(new RGBA_cloud);

  m_displayCloud->is_dense = false;
}

//---------------------------------------------------------
// Destructor

GridViewer::~GridViewer()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool GridViewer::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;

  for(p=NewMail.begin(); p!=NewMail.end(); p++){

    CMOOSMsg &msg = *p;
    string key  = msg.GetKey();
    string sval = msg.GetString();

    // Process lidar messages
    if (strEnds(key, "_DATA")) {

      size_t numBytes  = msg.GetBinaryDataSize();
      size_t numFloats = numBytes/sizeof(float);
      size_t numPoints = numFloats/4;
      // size_t numPoints = numBytes/sizeof(RGBA_point); 32 bytes!

      // Working cloud object - needs to be a shared pointer
      RGBA_cloud_ptr cloud (new RGBA_cloud());

      cloud->points.clear();
      cloud->width    = numPoints;
      cloud->height   = 1;
      cloud->is_dense = false;
      cloud->points.resize(numPoints);

      // Pointer to float message data
      float *ptr = reinterpret_cast<float *>(msg.GetBinaryData());

      // Create buffer for point data
      // RGBA_point * point_ptr = reinterpret_cast<RGBA_point *>(msg.GetBinaryData());

      // Copy data to point cloud
      for (size_t i = 0, j = 0; i < numPoints; ++i){
        
        cloud->points[i].x = ptr[j++];
        cloud->points[i].y = ptr[j++];
        cloud->points[i].z = ptr[j++];
        cloud->points[i].rgba = *reinterpret_cast<uint32_t*>(&ptr[j++]);
        
        // cloud->points[i].x = point_ptr->x;
        // cloud->points[i].y = point_ptr->y;
        // cloud->points[i].z = point_ptr->z;
        // cloud->points[i].rgba = point_ptr->rgba;
        // ++point_ptr;
      }

      m_octree->setInputCloud(cloud);
      m_octree->addPointsFromInputCloud();
    }

    else if (key == "LIDAR_VIEW")

      try {

        // Parse the json string
        json data = json::parse(sval);

        // Get camera name
        m_camera = data["camera"];

        // Camera position
        m_pos_x = data["pos"][0];
        m_pos_y = data["pos"][1];
        m_pos_z = data["pos"][2];

        // Camera orientation
        m_up_x = data["Y"][0];
        m_up_y = data["Y"][1];
        m_up_z = data["Y"][2];

        float view_x = data["Z"][0];
        float view_y = data["Z"][1];
        float view_z = data["Z"][2];

        // Incorrect documentation!
        // The view vector is actually the focus
        // point, so we need to explicitly compute
        // the camera's view vector like this...
        m_focus_x = m_pos_x + 100.f * view_x;
        m_focus_y = m_pos_y + 100.f * view_y;
        m_focus_z = m_pos_z + 100.f * view_z;

        m_camera_update = true;
      } catch (...) {

        cout << "Warning: invalid camera data!" << endl;

        m_camera_update = false;
      }
  }

  return true;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool GridViewer::OnConnectToServer()
{
   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool GridViewer::Iterate()
{
  m_iterations++;

  m_displayCloud->points.clear();

  // Start a display timer
  double start = MOOSTime();

  if (!m_auto_level) {

    // Display at full resolution
    m_display_depth = m_octree->getTreeDepth();
  }

  pcl::octree::OctreePointCloudOccupancy<RGBA_point>::Iterator it(&*m_octree);
  RGBA_point pt;

  while(*++it) {

    // Only process voxels at the display depth
    if (static_cast<int> (it.getCurrentOctreeDepth ()) != m_display_depth)
      continue;

      Eigen::Vector3f voxel_min, voxel_max;
      m_octree->getVoxelBounds(it, voxel_min, voxel_max);

      // Construct a point cloud
      pt.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
      pt.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
      pt.z = (voxel_min.z() + voxel_max.z()) / 2.0f;
      m_displayCloud->points.push_back(pt);

      // Don't iterate any lower
      it.skipChildVoxels();
  }

  // The code below should be faster
  // It appears that we must await Version 1.91 for the FixedDepthIterator, however.
  // pcl::octree::OctreePointCloudOccupancy<pcl::PointXYZ>::OctreeFixedDepthIterator it(&*m_octree, m_display_depth);
  // pcl::PointXYZ pt;

  // while(*++it) {

  //     Eigen::Vector3f voxel_min, voxel_max;
  //     m_octree->getVoxelBounds(it, voxel_min, voxel_max);

  //     // Construct a point cloud
  //     pt.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
  //     pt.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
  //     pt.z = (voxel_min.z() + voxel_max.z()) / 2.0f;
  //     m_displayCloud->points.push_back(pt);
  // }

  // The code below returns all of the leaf node centers
  // Ideally, we only want to display those at a certain level, however.
  // std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > voxelCenters;
  // int Nvox = m_octree->getOccupiedVoxelCenters(voxelCenters);

  // m_displayCloud->points.reserve(Nvox);
  // m_displayCloud->width  = Nvox;
  // m_displayCloud->height = 1;

  // // Copy voxel centers to display cloud
  // for (int i = 0; i < Nvox; i++)
  //   m_displayCloud->points.push_back(voxelCenters[i]);

  // Set the colour handler - not really ideal, as it needs to be inverted...
  pcl::visualization::PointCloudColorHandlerGenericField<RGBA_point> color_handler(m_displayCloud,"z");

  // Update display point cloud
  m_viewer->updatePointCloud<RGBA_point> (m_displayCloud, color_handler, "cloud");

  if (m_camera_update) {

    m_viewer->setCameraPosition(m_pos_x, m_pos_y, m_pos_z,
                                m_focus_x, m_focus_y, m_focus_z,
                                m_up_x, m_up_y, m_up_z);

    m_viewer->updateText(m_camera, 0, 20, 12, 1.0, 1.0, 1.0, "camera");
  } else
    m_viewer->updateText(m_camera + " (No data)", 0, 20, 12, 1.0, 1.0, 1.0, "camera");

  m_viewer->spinOnce(1,true);

  double display_time = MOOSTime() - start;

  // Automatically set display depth
  if (m_auto_level) {

    double allowed_time = 1.0 / GetAppFreq();

    // Is the display keeping up?
    if (display_time > 2.0 * allowed_time)
      m_down_count++;

    // Decrement display depth if too slow
    if (m_down_count > 10) {
      m_down_count = 0;
      m_display_depth--;
    }

    // Increment display depth if we are running fast enough
    if (display_time < 0.5 * allowed_time && m_display_depth < m_octree->getTreeDepth())
      m_up_count++;

    if (m_up_count > 10) {
      m_up_count = 0;
      m_display_depth++;
    }
  }

  cout << "Tree depth = " << m_octree->getTreeDepth() <<
        ", Disp depth = " << m_display_depth <<
        ", Disp time =  " << display_time <<
        ", Leaf count = " << m_octree->getLeafCount() << endl;

  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool GridViewer::OnStartUp()
{
  SetAppFreq(10,100);
  // SetIterateMode(COMMS_DRIVEN_ITERATE_AND_MAIL);

  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);

  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {

    list<string>::iterator p;

    for(p=sParams.begin(); p!=sParams.end(); p++) {

      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);
      double dval  = atof(value.c_str());
      bool   bval  = (strcasecmp (value.c_str(), "TRUE") == 0 || dval != 0);

      if (param == "LIDAR_FRAME") {

        // TO DO: Handle incoming data in lidar-frame
        cout << "LIDAR_FRAME = " << bval << endl;
        m_lidar_frame = bval;
      }
      else if (param == "RESOLUTION") {
        cout << "RESOLUTION = " << dval << endl;
        m_resolution = dval;
      }
      else if (param == "POINT_SIZE") {
        cout << "POINT_SIZE = " << dval << endl;
        m_point_size = dval;
      }
      else if(param == "FOV_DEG") {
        cout << "FOV_DEG = " << dval << endl;
        m_fov_deg = dval;
      }
      else if(param == "AUTO_LEVEL") {
        cout << "AUTO_LEVEL = " << bval << endl;
        m_auto_level = bval;
      }
    }
  }

  m_timewarp = GetMOOSTimeWarp();

  // look for latitude, longitude global variables
  double latOrigin, longOrigin;

  if(!m_MissionReader.GetValue("LatOrigin", latOrigin)) {

    MOOSTrace("pGridViewer: LatOrigin not set in *.moos file.\n");
    m_geo_ok = false;
  }
  else if(!m_MissionReader.GetValue("LongOrigin", longOrigin)) {

    MOOSTrace("pGridViewer: LongOrigin not set in *.moos file\n");
    m_geo_ok = false;
  }
  else {

    m_geo_ok = true;

    // initialize m_geodesy
    if(!m_geodesy.Initialise(latOrigin, longOrigin)) {
      MOOSTrace("pGridViewer: Geodesy init failed.\n");
      m_geo_ok = false;
    }
  }

  RegisterVariables();

  // Set the colour handler
  pcl::visualization::PointCloudColorHandlerGenericField<RGBA_point> color_handler(m_displayCloud,"z");

  // Set some viewer properties
  m_viewer->setCameraFieldOfView(m_fov_deg / 180.0 * M_PI);
  m_viewer->addText(m_camera, 0, 20, "camera");
  m_viewer->addPointCloud<RGBA_point> (m_displayCloud, color_handler, "cloud");
  m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, m_point_size, "cloud");

  // Override default resolution
  m_octree->setResolution(m_resolution);

  return true;
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void GridViewer::RegisterVariables()
{
  // Register for all Lidar data  
  Register("*_DATA", "pLidarSim", 0);

  // Point cloud view position and vector 
  Register("LIDAR_VIEW", 0);
}
