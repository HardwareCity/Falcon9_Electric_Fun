#include "pclviewer.h"
#include "../build/falcon9/ui_pclviewer.h"

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
  
  minDist = 500;
  maxDist = 5000;
/*
  blue  = 128;

  // Fill the cloud with some points
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);

    cloud->points[i].r = 128;
    cloud->points[i].g = 128;
    cloud->points[i].b = blue;
  }
*/

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));

  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  // Connect "random" button and the function
  connect (ui->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));

  // Connect R,G,B sliders and their functions
  connect (ui->horizontalSlider_min, SIGNAL (valueChanged (int)), this, SLOT (minSliderValueChanged (int)));
  connect (ui->horizontalSlider_max, SIGNAL (valueChanged (int)), this, SLOT (maxSliderValueChanged (int)));
  connect (ui->horizontalSlider_B, SIGNAL (valueChanged (int)), this, SLOT (blueSliderValueChanged (int)));
  //connect (ui->horizontalSlider_min, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
  //connect (ui->horizontalSlider_G, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
  //connect (ui->horizontalSlider_B, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));

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
  pass.setFilterLimits (minDist/1000.0, maxDist/1000.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  viewer->updatePointCloud(cloud_filtered, "kinect");
  ui->qvtkWidget->update();
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
PCLViewer::blueSliderValueChanged (int value)
{
  blue = value;
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
