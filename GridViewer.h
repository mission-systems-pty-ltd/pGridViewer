/************************************************************/
/*    NAME: David Battle                                    */
/*    ORGN: MISSION SYSTEMS PTY LTD                         */
/*    FILE: GridViewer.h                                    */
/*    DATE: 17 Dec 2018                                     */
/************************************************************/

#ifndef GridViewer_HEADER
#define GridViewer_HEADER

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree.h>
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "MOOS/libMOOS/MOOSLib.h"

typedef pcl::PointXYZRGBA RGBA_point;
typedef pcl::PointCloud<RGBA_point> RGBA_cloud;
typedef RGBA_cloud::Ptr RGBA_cloud_ptr;

class GridViewer : public CMOOSApp
{
 public:
   GridViewer();
   ~GridViewer();

   // Max simultaneous point clouds
   float m_altitude;
   float m_resolution;
   bool m_lidar_frame;
   bool m_auto_level;
   int m_point_size;
   int m_display_depth;
   int m_down_count;
   int m_up_count;

   // Camera parameters
   std::string m_camera;
   float m_pos_x, m_pos_y, m_pos_z;
   float m_focus_x, m_focus_y, m_focus_z;
   float m_up_x, m_up_y, m_up_z;
   bool  m_camera_update;
   float m_fov_deg;

 protected:
   bool OnNewMail( MOOSMSG_LIST &NewMail );
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();

   CMOOSGeodesy m_geodesy;
   bool         m_geo_ok;

 private:

   // PCL Visualizer object
   boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;

   // Display cloud object
   RGBA_cloud_ptr m_displayCloud;

   // Octree for storing occupancy grid
   boost::shared_ptr<pcl::octree::OctreePointCloudOccupancy<RGBA_point>> m_octree;

   unsigned int m_iterations;
   double       m_timewarp;
};

#endif
