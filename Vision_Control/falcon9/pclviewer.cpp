#include "pclviewer.h"
#include "../build/falcon9/ui_pclviewer.h"

#include <pcl/visualization/point_picking_event.h> 

#define OBJECT_SIZE_WIDTH 0.07
#define OBJECT_SIZE_HEIGHT 0.15
#define OBJECT_SEARCH_FACTOR 1.1
//#define TEST_RIG 1

#define MOTION_TICK_SECS 0.033

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
  //this->setWindowTitle ("PCL viewer");

  // Setup the cloud pointer
  cloud.reset (new PointCloudT);
  // The number of points in the cloud
  cloud->points.resize (200);

  kf.reset();

  // The default color
  
  minDist = 1200;
  maxDist = 2100;
  minHeight = 100;

  ui->horizontalSlider_min->setValue(minDist);
  minSliderValueChanged(minDist);

  ui->horizontalSlider_max->setValue(maxDist);
  maxSliderValueChanged(maxDist);

  ui->horizontalSlider_minHeight->setValue(minHeight);
  minHeightSliderValueChanged(minHeight);

  picked_event = false;
  run = false;

  // Set up the QVTK windows
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  viewer->registerPointPickingCallback (pp_callback,(void*)this);

  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  viewer->setShowFPS(false);
  ui->qvtkWidget->update ();


  viewer2.reset (new pcl::visualization::PCLVisualizer ("viewer2", false));
  ui->qvtkWidget_2->SetRenderWindow (viewer2->getRenderWindow ());
  viewer2->setupInteractor (ui->qvtkWidget_2->GetInteractor (), ui->qvtkWidget_2->GetRenderWindow ());
  viewer2->setShowFPS(false); 
  ui->qvtkWidget_2->update ();

  // Connect "random" button and the function
  connect (ui->pushButton_save,  SIGNAL (clicked ()), this, SLOT (saveButtonPressed ()));
  connect (ui->pushButton_setOrigin,  SIGNAL (clicked ()), this, SLOT (setOrigin()));

  // Connect R,G,B sliders and their functions
  connect (ui->horizontalSlider_min, SIGNAL (valueChanged (int)), this, SLOT (minSliderValueChanged (int)));
  connect (ui->horizontalSlider_max, SIGNAL (valueChanged (int)), this, SLOT (maxSliderValueChanged (int)));
  connect (ui->horizontalSlider_minHeight, SIGNAL (valueChanged (int)), this, SLOT (minHeightSliderValueChanged (int)));

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

  pcl::PointXYZ p0;
  p0.x = 0.0;
  p0.y = 0.0;
  p0.z = 0.0;
  viewer2->addSphere(p0, OBJECT_SIZE_WIDTH, 1.0, 0.0, 0.0, "rocket");

  viewer2->addCoordinateSystem (1.0);

  // Viewer2
  viewer2->addText("Raw position: ...", 10, 10, "rawPos");
  viewer2->addText("KF position: ...", 10, 20, "kfPos");
  viewer2->addText("Rocket position: ...", 10, 30, "rocketPos");


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


  if(run || picked_event)
  {
    run = true;

    // Print picked point
    std::cout << pickedPoint.x << " ; " << pickedPoint.y << " ; " << pickedPoint.z << std::endl;

    // Save last position
    Eigen::Vector3f lastPosition = kf.getPos();

    if(picked_event) // first time, force Kalman Filter state
    {
      kf.reset();
      Eigen::Vector3f initPos;
      initPos << kinect_base_position(0) + pickedPoint.x,
       kinect_base_position(1) - pickedPoint.y,
       kinect_base_position(2) - pickedPoint.z;

      kf.setPos(initPos);
    }
    else
    {
      // Kalman Filter predict step
      Matrix6f QAbs; // Process noise
      QAbs <<   
        0.05, 0,    0,      0,    0,    0,
        0,    0.05, 0,      0,    0,    0,
        0,    0,    0.05,   0,    0,    0,
        0,    0,    0,      0.5,  0,    0,
        0,    0,    0,      0,    0.5,  0,
        0,    0,    0,      0,    0,    0.5;
      kf.predict(MOTION_TICK_SECS, QAbs);
    }

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

    // Weighted centroid tracking

    // calculate median distance

    if(cloud_roi->points.size() > 10)
    {

      // Step 1: calculate median distance
      std::vector<float> distances;
      distances.resize(cloud_roi->points.size());
      pcl::PointCloud<pcl::PointXYZRGB>::iterator b1;
      for (b1 = cloud_roi->points.begin(); b1 < cloud_roi->points.end(); b1++)
        distances.push_back(b1->z);

      float medianDistance = distances[distances.size()/2];

      // Step 2: 
      pcl::PointXYZ centroid(0,0,0);
      float cost_aux = 0.0; // cost grows exponentially regarding the median (=1 in the median, =0 when distance is 0)
      float parabolla_a = 1.0/(medianDistance*medianDistance);
      float total_weight = 0.0f;
      for (b1 = cloud_roi->points.begin(); b1 < cloud_roi->points.end(); b1++)
      {
        float zVal = b1->z;
        if(zVal > medianDistance)
          zVal *= 2.0;

        cost_aux = parabolla_a * zVal * zVal; // cost = a*x^2
        total_weight += 1.0 / cost_aux; // weight is inverse of cost

        centroid.x += b1->x / cost_aux;
        centroid.y += b1->y / cost_aux;
        centroid.z += b1->z / cost_aux;
      }

      centroid.x /= total_weight;
      centroid.y /= total_weight;
      centroid.z /= total_weight;

      pickedPoint.x = centroid.x;
      pickedPoint.y = centroid.y;
      pickedPoint.z = centroid.z;

      // Recalc cluster based on new picked point position

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

      Eigen::Vector3f pos;
      pos << kinect_base_position(0) + pickedPoint.x,
       kinect_base_position(1) - pickedPoint.y,
       kinect_base_position(2) - pickedPoint.z;

      // Kalman Filter update step
      if(!picked_event) // do not update on first cycle
      {
        
        
        Eigen::Vector3f vel = pos - lastPosition;

        // Measurement noise
        Matrix6f RAbs;
        RAbs <<   
          0.05, 0,    0,    0,    0,    0,
          0,    0.05, 0,    0,    0,    0,
          0,    0,    0.05, 0,    0,    0,
          0,    0,    0,    2.5,  0,    0,
          0,    0,    0,    0,    2.5,  0,
          0,    0,    0,    0,    0,    2.5;
        kf.update(pos, vel, MOTION_TICK_SECS, RAbs);
      }
      Eigen::Vector3f newPosition = kf.getPos();

      // Update pickpoint
      //pickedPoint.x = newPosition(0);
      //pickedPoint.y = newPosition(1);
      //pickedPoint.z = newPosition(2);

      pcl::PointXYZ p0;
      p0.x = newPosition(0);
      p0.y = newPosition(1);
      p0.z = newPosition(2);
      //add arrow between two center
      viewer2->updateSphere(p0, OBJECT_SIZE_WIDTH, 1.0, 0.0, 0.0, "rocket");


      viewer2->updateText(QString().sprintf("Raw position: (%6.2f,%6.2f,%6.2f)", pos(0), pos(1), pos(2)).toStdString(), 10, 10, "rawPos");
      viewer2->updateText(QString().sprintf("KF position: (%6.2f,%6.2f,%6.2f)", newPosition(0), newPosition(1), newPosition(2)).toStdString(), 10, 20, "kfPos");


      // Rocket position relative to rocket origin
      rocketPos = kf.getPos();
      rocketPos -= rocket_origin;
      viewer2->updateText(QString().sprintf("Rocket position: (%6.2f,%6.2f,%6.2f)", rocketPos(0), rocketPos(1), rocketPos(2)).toStdString(), 10, 30, "rocketPos");

      viewer2->updatePointCloud(cloud_roi, "kinect");
      ui->qvtkWidget_2->update();
    }

    // Set origin when setting new position
    if(picked_event)
      setOrigin();


    picked_event = false;
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
PCLViewer::saveButtonPressed ()
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

void PCLViewer::setOrigin()
{
  rocket_origin = kf.getPos();
  //rocket_origin(2) -= OBJECT_SIZE_HEIGHT/2;
}

PCLViewer::~PCLViewer ()
{
  k2g->shutDown();
  delete ui;
}
