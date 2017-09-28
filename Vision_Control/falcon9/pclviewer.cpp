#include "pclviewer.h"
#include "../build/falcon9/ui_pclviewer.h"

#include <pcl/visualization/point_picking_event.h> 

#define OBJECT_SIZE_WIDTH 0.07
#define OBJECT_SIZE_HEIGHT 0.15
#define OBJECT_SEARCH_FACTOR 1.1
//#define TEST_RIG 1

void
pp_callback(const pcl::visualization::PointPickingEvent& event, void*
viewer_void){
   std::cout << "Picking event active" << std::endl;
     if(event.getPointIndex()!=-1)
     {
         float x,y,z;
         event.getPoint(x,y,z);
         
         PCLViewer* window = (PCLViewer*)viewer_void;
         window->pickedPoint.x = x;
         window->pickedPoint.y = y;
         window->pickedPoint.z = z;
         window->picked_event = true;
     }
} 

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");

  // Setup the cloud pointer
  cloud.reset (new PointCloudT);
  // The number of points in the cloud
  cloud->points.resize (200);

  // The default color
  
  minDist = 1400;
  maxDist = 2300;
  minHeight = 100;

  picked_event = false;

  // Set up the QVTK windows
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  viewer->registerPointPickingCallback (pp_callback,(void*)this);

  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();


  viewer2.reset (new pcl::visualization::PCLVisualizer ("viewer2", false));
  ui->qvtkWidget_2->SetRenderWindow (viewer2->getRenderWindow ());
  viewer2->setupInteractor (ui->qvtkWidget_2->GetInteractor (), ui->qvtkWidget_2->GetRenderWindow ());
  ui->qvtkWidget_2->update ();

  // Connect "random" button and the function
  connect (ui->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));

  // Connect R,G,B sliders and their functions
  connect (ui->horizontalSlider_min, SIGNAL (valueChanged (int)), this, SLOT (minSliderValueChanged (int)));
  connect (ui->horizontalSlider_max, SIGNAL (valueChanged (int)), this, SLOT (maxSliderValueChanged (int)));
  connect (ui->horizontalSlider_minHeight, SIGNAL (valueChanged (int)), this, SLOT (minHeightSliderValueChanged (int)));

  // Connect point size slider
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

  //viewer->addPointCloud (cloud, "cloud");

  pSliderValueChanged (2);
  viewer->resetCamera ();
  ui->qvtkWidget->update ();



  // Kinect process
  
  Processor freenectprocessor = OPENGL;
  
  k2g = new K2G(freenectprocessor);
  cloud_new = k2g->getCloud();
  k2g->printParameters();


  cloud_new->sensor_orientation_.w() = 0.0;
  cloud_new->sensor_orientation_.x() = 1.0;
  cloud_new->sensor_orientation_.y() = 0.0;
  cloud_new->sensor_orientation_.z() = 0.0;


  kinect_base_position(0) = 0.0; // x
  kinect_base_position(1) = 1.0; // y
  kinect_base_position(2) = 0.0; // z
  kinect_base_position(3) = 0.0; // ?

  cloud_new->sensor_origin_ = kinect_base_position;


  // Show main axis
  viewer->addCoordinateSystem (1.0);

  viewer->addPointCloud (cloud_new, "kinect");
  viewer2->addPointCloud (cloud_new, "kinect");

  // Show coordinate system

  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(1.0);
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(0.0);
  viewer->addPlane (coeffs, "plane");


  Update_timer = new QTimer();
  connect(Update_timer, SIGNAL(timeout()), this, SLOT(update_cloud()));
  Update_timer->start(100);
}

void PCLViewer::update_cloud()
{
  cloud_new = k2g->getCloud();


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Create the filtering object
  
  pass.setInputCloud(cloud_new);
  pass.setFilterFieldName ("z");
#ifdef TEST_RIG
  pass.setFilterLimits (1.5, 2.3);
#else
  pass.setFilterLimits (minDist/1000.0, maxDist/1000.0);
#endif
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-100.0, kinect_base_position(1) - minHeight/1000.0);
  pass.filter (*cloud_filtered);

  viewer->updatePointCloud(cloud_filtered, "kinect");
  ui->qvtkWidget->update();


  if(picked_event)
  {
    // Print picked point
    std::cout << pickedPoint.x << " ; " << pickedPoint.y << " ; " << pickedPoint.z << std::endl;

    // Set region of interest around picked point
    float OBJECT_SEARCH_WIDHT = OBJECT_SIZE_WIDTH/2.0 * OBJECT_SEARCH_FACTOR;
    float OBJECT_SEARCH_HEIGHT = OBJECT_SIZE_HEIGHT/2.0 * OBJECT_SEARCH_FACTOR;


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_roi (new pcl::PointCloud<pcl::PointXYZRGB>);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (pickedPoint.x - OBJECT_SIZE_WIDTH , pickedPoint.x + OBJECT_SIZE_WIDTH);
    pass.filter (*cloud_roi);

    pass.setInputCloud(cloud_roi);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (pickedPoint.y - OBJECT_SEARCH_HEIGHT, pickedPoint.y + OBJECT_SEARCH_HEIGHT);
    pass.filter (*cloud_roi);

    pass.setInputCloud(cloud_roi);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (pickedPoint.z - OBJECT_SIZE_WIDTH, pickedPoint.z + OBJECT_SIZE_WIDTH);
    pass.filter (*cloud_roi);

    sor.setInputCloud (cloud_roi);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_roi);

    viewer2->updatePointCloud(cloud_roi, "kinect");
    ui->qvtkWidget_2->update();

    

/*
    pcl::MinCutSegmentation<pcl::PointXYZRGB> seg;
    seg.setInputCloud (cloud_roi);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZRGB> ());
    foreground_points->points.push_back(pickedPoint);
    seg.setForegroundPoints (foreground_points);

    seg.setSigma (maxDist/1000.0);
    seg.setRadius (2*OBJECT_SIZE_HEIGHT);
    seg.setNumberOfNeighbours (10);
    seg.setSourceWeight (minDist/1000.0);

    std::vector <pcl::PointIndices> clusters;
    seg.extract (clusters);

    std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
    viewer2->updatePointCloud(colored_cloud, "kinect");
    ui->qvtkWidget_2->update();
*/




    //picked_event = false;

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*cloud_roi, centroid);
    pickedPoint.x = centroid(0);
    pickedPoint.y = centroid(1);
    pickedPoint.z = centroid(2);
  }



/* // --- region growing over all points
  // -- TOO SLOW to do every cycle

  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud_filtered);
  //reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (0.2);
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (5);
  reg.setMinClusterSize (600);


  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  
  // Update viewer2
  viewer2->updatePointCloud(colored_cloud, "kinect");
  ui->qvtkWidget_2->update();
  */
}

void
PCLViewer::randomButtonPressed ()
{
  printf ("Random button was pressed\n");

  // Set the new color
  for (size_t i = 0; i < cloud->size(); i++)
  {
    cloud->points[i].r = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
    cloud->points[i].g = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
    cloud->points[i].b = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
  }

  viewer->updatePointCloud (cloud, "cloud");
}

void
PCLViewer::RGBsliderReleased ()
{
  /*
  // Set the new color
  for (size_t i = 0; i < cloud->size (); i++)
  {
    cloud->points[i].r = red;
    cloud->points[i].g = green;
    cloud->points[i].b = blue;
  }
  viewer->updatePointCloud (cloud, "cloud");
  */
}

void
PCLViewer::pSliderValueChanged (int value)
{
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
}

void
PCLViewer::minSliderValueChanged (int value)
{
  minDist = value;
  ui->lbl_min->setText(QString().sprintf("Min distance: %.1fm", value*1.0/1000.0));
}

void
PCLViewer::maxSliderValueChanged (int value)
{
  maxDist = value;
  ui->lbl_max->setText(QString().sprintf("Max distance: %.1fm", value*1.0/1000.0));
}

void
PCLViewer::minHeightSliderValueChanged (int value)
{
  minHeight = value;
  ui->lbl_minHeight->setText(QString().sprintf("Min height: %.1fm", value*1.0/1000.0));
}

PCLViewer::~PCLViewer ()
{
  k2g->shutDown();
  delete ui;
}
