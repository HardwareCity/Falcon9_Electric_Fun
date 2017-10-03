#include "pclviewer.h"
#include "../build/falcon9/ui_pclviewer.h"

#include <pcl/visualization/point_picking_event.h> 
#include <QSettings>
#include <QtSerialPort/QSerialPortInfo>

#define OBJECT_SIZE_WIDTH 0.10
#define OBJECT_SIZE_HEIGHT 1.18
//#define OBJECT_SIZE_HEIGHT 0.15
#define OBJECT_SEARCH_FACTOR 1.1
//#define TEST_RIG 1

#define MOTION_TICK_SECS 0.06

//#define OBJECT_ACTOR_CUBE 1
#define OBJECT_ACTOR_PCLOGO 1

//#define CAMERA_TILT 5 * M_PI/180
#define CAMERA_TILT 0

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
  ui (new Ui::PCLViewer),
  finalThrust(4)
{
  ui->setupUi (this);
  //this->setWindowTitle ("PCL viewer");

  // Load user settings
  m_sSettingsFile = "demosettings.ini";
  loadSettings();
  pid.setMaxOut(1.0);

  // Update UI with loaded settings
  ui->horizontalSlider_min->setValue(minDist);
  minSliderValueChanged(minDist);

  ui->horizontalSlider_max->setValue(maxDist);
  maxSliderValueChanged(maxDist);

  ui->horizontalSlider_minHeight->setValue(minHeight);
  minHeightSliderValueChanged(minHeight);

  ui->doubleSpin_kp->setValue(pid.getP());
  ui->doubleSpin_ki->setValue(pid.getI());
  ui->doubleSpin_kd->setValue(pid.getD());
  ui->doubleSpin_maxint->setValue(pid.getMaxInt());

  controlOutput = 0.0f;
  picked_event = false;
  run = false;
  showFog = false;
  finalThrustIntegral = 0.0f;
  textSide = NULL;
  applyPID = false;
  countDownTimer = 5999;

  // Setup the cloud pointer
  cloud.reset (new PointCloudT);
  // The number of points in the cloud
  cloud->points.resize (200);

  kf.reset();

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

  // Connect buttons and the functions
  connect (ui->pushButton_save,  SIGNAL (clicked ()), this, SLOT (saveButtonPressed ()));
  connect (ui->pushButton_setOrigin,  SIGNAL (clicked ()), this, SLOT (setOrigin()));
  connect (ui->pushButton_showFog,  SIGNAL (toggled(bool)), this, SLOT (showPixelFog(bool)));
  connect (ui->pushButton_connect,  SIGNAL (toggled(bool)), this, SLOT (connectSerial(bool)));

  // Connect sliders and their functions
  connect (ui->horizontalSlider_min, SIGNAL (valueChanged (int)), this, SLOT (minSliderValueChanged (int)));
  connect (ui->horizontalSlider_max, SIGNAL (valueChanged (int)), this, SLOT (maxSliderValueChanged (int)));
  connect (ui->horizontalSlider_minHeight, SIGNAL (valueChanged (int)), this, SLOT (minHeightSliderValueChanged (int)));

  // Connect PID parameters
  connect (ui->doubleSpin_kp, SIGNAL (valueChanged (double)), this, SLOT (kpChanged(double)));
  connect (ui->doubleSpin_ki, SIGNAL (valueChanged (double)), this, SLOT (kiChanged(double)));
  connect (ui->doubleSpin_kd, SIGNAL (valueChanged (double)), this, SLOT (kdChanged(double)));
  connect (ui->doubleSpin_maxint, SIGNAL (valueChanged (double)), this, SLOT (maxIntChanged(double)));

  pSliderValueChanged (2);
  viewer->resetCamera ();
  ui->qvtkWidget->update ();



  // Kinect process
  
  Processor freenectprocessor = OPENGL;
  
  k2g = new K2G(freenectprocessor);
  cloud_new = k2g->getCloud();
  k2g->printParameters();


  cloud_new->sensor_orientation_.w() = CAMERA_TILT;
  cloud_new->sensor_orientation_.x() = 1.0;
  cloud_new->sensor_orientation_.y() = 0.0;
  cloud_new->sensor_orientation_.z() = 0.0;


  kinect_base_position(0) = 0.0; // x
  kinect_base_position(1) = 2.0; // y
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



  // -------
  // AR View

  renWin = vtkRenderWindow::New();
  ui->qvtkWidget_3->SetRenderWindow (renWin);

  backgroundRenderer = vtkRenderer::New();
  counterRenderer = vtkRenderer::New();
  sceneRenderer = vtkRenderer::New();
  importer = vtkImageImport::New();
  imageActor = vtkImageActor::New();
  imageFlip = vtkImageFlip::New();



  importer->SetWholeExtent(0, 1920-1, 0, 1080-1, 0, 0);    // 640x480 image...
  importer->SetDataExtentToWholeExtent();
  importer->SetNumberOfScalarComponents( 3 );        // 3 for R G B A ....    importer->SetDataSpacing(1,1,1);
  importer->SetDataScalarTypeToUnsignedChar();
  

  // Create a dummy image to show on startup
  cv::Mat dummyMat = cv::Mat(1080, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
  importer->SetImportVoidPointer((void * ) dummyMat.data, 1);

  // Flip Image
  imageFlip->SetInput(NULL); // refresh pointer to force render again
  imageFlip->SetInput(importer->GetOutput());
  renWin->Render();
  imageFlip->Update();
  imageFlip->SetFilteredAxis(1);         // inverted Y axis
  


  imageActor->SetInput( imageFlip->GetOutput() );
  imageActor->SetDisplayExtent(0, 1920-1, 0, 1080-1, 0, 0);


  // Add Counter
  counterActor = vtkTextActor::New();
  counterActor->SetInput ( "5...." );
  counterActor->SetPosition2 ( 100, 100 );
  counterActor->GetTextProperty()->SetFontSize ( 340 );
  counterActor->GetTextProperty()->SetColor ( 1.0, 1.0, 1.0 );
  counterRenderer->AddActor2D ( counterActor );


  backgroundRenderer->SetLayer(0);
  counterRenderer->SetLayer(1);
  sceneRenderer->SetLayer(2);

  renWin->SetNumberOfLayers(3);
  renWin->AddRenderer(backgroundRenderer);
  renWin->AddRenderer(counterRenderer);
  renWin->AddRenderer(sceneRenderer);
  renWin->SetSize(1920,1080);

  backgroundRenderer->AddActor2D( imageActor );

  cubeActor = vtkActor::New();
  cubeSource = vtkCubeSource::New();
#ifdef OBJECT_ACTOR_CUBE
  // Create cube to show on AR view
  vtkCubeSource* cubeSource = vtkCubeSource::New();
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInput(cubeSource->GetOutput());
  cubeActor->SetMapper(mapper);
  cubeActor->SetScale(0.1);
  cubeActor->SetPosition(0,0,25);
  sceneRenderer->AddActor(cubeActor);
#endif
#ifdef OBJECT_ACTOR_PCLOGO
  
  vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
  reader->SetFileName("../3d_models/pixelscamp_rocket.stl");
  reader->Update();
 
  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(reader->GetOutputPort());
 

  cubeActor->SetMapper(mapper);
  cubeActor->SetScale(0.005);
  cubeActor->SetPosition(0,0,25);
  cubeActor->SetOrigin(-0.14,0,0);
  cubeActor->GetProperty()->SetColor(233/255.0, 75/255.0, 53/255.0);
  sceneRenderer->AddActor(cubeActor);
#endif

  vtkCamera* camera = sceneRenderer->GetActiveCamera();
  double camPos[3];
  camPos[0] = kinect_base_position(0);
  camPos[1] = kinect_base_position(1);
  camPos[2] = kinect_base_position(2);
  camera->SetPosition( camPos );

  double lookat[3];
  lookat[0] = kinect_base_position(0);
  lookat[1] = kinect_base_position(1) - sin(CAMERA_TILT);
  lookat[2] = kinect_base_position(2) - cos(CAMERA_TILT);
  camera->SetFocalPoint( lookat );

  double viewUp[3];
  viewUp[0] = 0;
  viewUp[1] = 1;
  viewUp[2] = 0;
  camera->SetViewUp( viewUp );
  
  camera->SetClippingRange(0.01, 10000.0);
  camera->SetViewAngle(43);
  //return;
  
  //ui->qvtkWidget_3->update ();

  // ------

  Update_timer = new QTimer();
  connect(Update_timer, SIGNAL(timeout()), this, SLOT(update_cloud()));
  Update_timer->start(100);

  sendSerialTimer = new QTimer();
  connect(sendSerialTimer, SIGNAL(timeout()), this, SLOT(sendSerial()));
  sendSerialTimer->start(100);

  //cv::namedWindow("cvwindow");
}

void PCLViewer::sendSerial()
{
  

  finalThrustIntegral += controlOutput*10.0;
  if(finalThrustIntegral > 100)
    finalThrustIntegral = 100;
  if(finalThrustIntegral < 0)
    finalThrustIntegral = 0;

  // Hack base + PID
  float controlOutputFloat = controlOutput*10.0;
  if(!applyPID)
  {
    if(controlOutputFloat < -1)
      applyPID = true;
  }
  finalThrustIntegral = ui->slider_current->value();

  if(applyPID)
    finalThrustIntegral += controlOutputFloat;

  fprintf(stdout, "FINAL: %.2f + %.2f = %.2f\n", ui->slider_current->value()*1.0, controlOutputFloat, finalThrustIntegral);
  finalThrust.addValue(finalThrustIntegral);
  

  if(ui->pushButton_connect->isChecked())
  {
    int val = finalThrust.mean();
    if(val > 100) val = 100;
    if(val < 0) val = 0;
    //unsigned char sendVal = (unsigned char)val;
    //serial.write(sendVal);
    //serial.write(QString().sprintf("%d\n", finalThrust.mean()));

    QByteArray ba;
    ba.resize(2);
    ba[0] = val + 1;
    ba[1] = 0x00;

    serial.write(ba);
    //fprintf(stdout,"Sending...\n");

    countDownTimer -= 110;
  }else{
    // Not connected, zero the final thrust
    finalThrustIntegral = 0;
    countDownTimer = 5999;
    applyPID = false;
  }
  
  int finalThrustFiltered = finalThrust.mean();
  ui->thrustOut->setValue(finalThrustFiltered);
  this->showFog = finalThrustFiltered > 10;
}

void PCLViewer::update_cloud()
{
  int width = 512; int height = 424;
  cv::Mat tmp_depth;
  cv::Mat tmp_color;

  k2g->get(tmp_color, tmp_depth, cloud_new);

  cv::Mat tmp_color_3bytes;
  cv::cvtColor(tmp_color,tmp_color_3bytes,cv::COLOR_BGRA2RGB);


  int roiW = tmp_color_3bytes.size().width * 0.8;
  int roiH = tmp_color_3bytes.size().height * 0.8;

  cv::Rect roi;
  roi.x = tmp_color_3bytes.size().width/2 - roiW/2;
  roi.y = tmp_color_3bytes.size().height/2 - roiH/2;
  roi.width = roiW;
  roi.height = roiH;

  /* Crop the original image to the defined ROI */
  cv::Mat crop = tmp_color_3bytes(roi);

  cv::Size size(1920,1080);
  cv::Mat croppedFullHD;
  cv::resize(crop, croppedFullHD, size);//resize image

  // Background 
  if(!ui->pushButton_connect->isChecked() || countDownTimer > 0)
  {
    croppedFullHD *= 0.1;//cv::Scalar(0,0,0);
  }



  //cv::imshow("cvwindow", croppedFullHD);


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

    

    // Calculate control output
    float target = (float)(ui->slider_target->value() / 1000.0);
    controlOutput = pid.compensate(target - rocketPos(1));
    fprintf(stdout,"error = %.2f , controlOutput = %.2f\n", target - rocketPos(1), controlOutput);
    ui->throttle_pos->setValue(controlOutput*100);

    // Update sliders and labels
    //ui->slider_current->setValue(rocketPos(1) * 1000.0);
    ui->lbl_height_current->setText(QString().sprintf("%.2f", rocketPos(1)));
    ui->lbl_height_target->setText(QString().sprintf("%.2f", target));

    // Set origin when setting new position
    if(picked_event)
      setOrigin();


    picked_event = false;


    // -------
    // AR View

    // Set the data pointer to importer
    importer->SetImportVoidPointer((void * ) croppedFullHD.data, 1);

    // Flip Image
    imageFlip->SetInput(NULL); // refresh pointer to force render again
    imageFlip->SetInput(importer->GetOutput());
    renWin->Render();
    imageFlip->Update();

    // Set up the background camera to fill the renderer with the image
    double origin[3];
    double spacing[3];
    int extent[6];
    imageFlip->GetOutput()->GetOrigin( origin );
    imageFlip->GetOutput()->GetSpacing( spacing );
    imageFlip->GetOutput()->GetExtent( extent );

    // Display camera image in background layer
    vtkCamera* camera = backgroundRenderer->GetActiveCamera();
    camera->ParallelProjectionOn();

    double xc = origin[0] + 0.5*(extent[0] + extent[1])*spacing[0];
    double yc = origin[1] + 0.5*(extent[2] + extent[3])*spacing[1];
    //double xd = (extent[1] - extent[0] + 1)*spacing[0];
    double yd = (extent[3] - extent[2] + 1)*spacing[1];
    double d = camera->GetDistance();
    camera->SetParallelScale(0.5*yd);
    camera->SetFocalPoint(xc,yc,0.0);
    camera->SetPosition(xc,yc,d);

    // Set the cube position
    Eigen::Vector3f cubePosition_KF = kf.getPos();
    Eigen::Vector3f cubePosition;
    cubePosition << kinect_base_position(0) + pickedPoint.x,
     kinect_base_position(1) - pickedPoint.y,
     kinect_base_position(2) - pickedPoint.z;

    cubeActor->SetPosition(cubePosition(0), cubePosition(1), cubePosition(2));

    if(!ui->pushButton_connect->isChecked() || countDownTimer > 0)
    {
      cubeActor->GetProperty()->SetOpacity(0.3);
    }else{
      cubeActor->GetProperty()->SetOpacity(1.0);
    }

    if(textSide)
    {
      // Delete last text
      sceneRenderer->RemoveActor(textSide);
      textSide->Delete();
    }

    // Create new text
    textSide = createText(QString().sprintf("Height: %.1f\nThrust: %d%%", rocketPos(1), ui->thrustOut->value()));
    textSide->GetProperty()->SetAmbient(1.0);
    textSide->GetProperty()->SetColor(1.0,1.0,1.0);
    //textSide->SetOrientation(0,0,0);
    textSide->SetPosition(cubePosition(0) + 0.4, cubePosition(1), cubePosition(2));
    textSide->SetScale(0.07);
    if(!ui->pushButton_connect->isChecked() || countDownTimer > 0)
    {
      textSide->GetProperty()->SetOpacity(0.3);
    }else{
      textSide->GetProperty()->SetOpacity(1.0);
    }
    sceneRenderer->AddActor(textSide);

    camera = sceneRenderer->GetActiveCamera();
    double camPos[3];
#ifdef OBJECT_ACTOR_CUBE
    camPos[0] = kinect_base_position(0) - 0.05; // Offset between depth and RGB
    camPos[1] = kinect_base_position(1) - 0.05;
    camPos[2] = kinect_base_position(2);
#endif
#ifdef OBJECT_ACTOR_PCLOGO
    camPos[0] = kinect_base_position(0); // Offset between depth and RGB
    camPos[1] = kinect_base_position(1) - 0.04;
    camPos[2] = kinect_base_position(2);
#endif
    camera->SetPosition( camPos );

    double lookat[3];
    lookat[0] = kinect_base_position(0);
    lookat[1] = kinect_base_position(1) - sin(CAMERA_TILT);
    lookat[2] = kinect_base_position(2) - cos(CAMERA_TILT);
    camera->SetFocalPoint( lookat );

    double viewUp[3];
    viewUp[0] = 0;
    viewUp[1] = 1;
    viewUp[2] = 0;
    camera->SetViewUp( viewUp );
    
    camera->SetClippingRange(0.01, 10000.0);
    camera->SetViewAngle(43);
  
    // Update Counter
    int counterInt = (int)(countDownTimer/1000.0);
    if(counterInt >= 0)
    {
      //if(counterInt <= 0)
        counterActor->SetInput ( QString().sprintf("%d...", counterInt).toStdString().c_str() );
      //else
      //  counterActor->SetInput ( "GO");

      counterActor->GetProperty()->SetOpacity(1);
    }else{
      counterActor->GetProperty()->SetOpacity(0);
    }
    

    // FOG
    updatePixelFog(MOTION_TICK_SECS);

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
  saveSettings();
  fprintf(stdout,"SETTINGS SAVED!\n");
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

void PCLViewer::showPixelFog(bool active) {
  fprintf(stderr,"Showfog: %d\n", active?1:0);
  
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

void PCLViewer::kpChanged(double val) {
  pid.setP(val);
}

void PCLViewer::kiChanged(double val) {
  pid.setI(val);
}

void PCLViewer::kdChanged(double val) {
  pid.setD(val);
}

void PCLViewer::maxIntChanged(double val) {
  pid.setMaxInt(val);
}

void PCLViewer::setOrigin()
{
  rocket_origin = kf.getPos();
  ui->slider_target->setValue(100);
  //rocket_origin(2) -= OBJECT_SIZE_HEIGHT/2;
}

PCLViewer::~PCLViewer ()
{
  Update_timer->stop();
  k2g->shutDown();
  delete ui;
}


void PCLViewer::loadSettings()
{
  QSettings settings(m_sSettingsFile, QSettings::NativeFormat);
  
  minDist = settings.value("minDist", 1200).toUInt();
  maxDist = settings.value("maxDist", 2100).toUInt();
  minHeight = settings.value("minHeight", 100).toUInt();

  // PID Settings loaded to PID object directly
  pid.setP(settings.value("Kp", 1.0).toFloat());
  pid.setI(settings.value("Ki", 0.0).toFloat());
  pid.setD(settings.value("Kd", 0.0).toFloat());
  pid.setMaxInt(settings.value("maxInt", 0.0).toFloat());
}

void PCLViewer::saveSettings()
{
  QSettings settings(m_sSettingsFile, QSettings::NativeFormat);
  settings.setValue("minDist", minDist);
  settings.setValue("maxDist", maxDist);
  settings.setValue("minHeight", minHeight);

  // PID Settings
  settings.setValue("Kp", (double)pid.getP());
  settings.setValue("Ki", (double)pid.getI());
  settings.setValue("Kd", (double)pid.getD());
  settings.setValue("maxInt", (double)pid.getMaxInt());
}

void PCLViewer::updatePixelFog(float deltaT) {
  // Update elements
  std::vector<FogPixel>::iterator it = fogElements.begin();
  while(it != fogElements.end())
  {
    // check death
    if(it->age >= it->death)
    {
      sceneRenderer->RemoveActor(it->actor);
      it = fogElements.erase(it);
      continue;
    }
    
    // Iterate position
    it->pos = it->pos + deltaT * it->vel;
    if(it->pos[1] < 0)
    {
      it->pos[1] = 0;
      it->vel[1] = 0;
    }
    it->actor->SetPosition(it->pos[0], it->pos[1], it->pos[2]);

    // Set actor opacity
    float opacity = it->opacityRamp.getValue(it->age);
    //fprintf(stderr,"opacity: %.2f\n", opacity);
    it->actor->GetProperty()->SetOpacity(opacity);

    // Increment age
    it->age++;

    // Iterate next pixel
    it++;
  }

  // Create fog particles
  if(showFog)
  {
    fogElements.push_back(createFogPixel());
    fogElements.push_back(createFogPixel());
    fogElements.push_back(createFogPixel());
    fogElements.push_back(createFogPixel());
  }
}

FogPixel PCLViewer::createFogPixel()
{
  FogPixel ret;

  vtkActor* fogActor = vtkActor::New();
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInput(cubeSource->GetOutput());
  fogActor->SetMapper(mapper);
  fogActor->SetScale(0.05);
  
  Eigen::Vector3f cubePosition_KF = kf.getPos();
  Eigen::Vector3f cubePosition;
  cubePosition << kinect_base_position(0) + pickedPoint.x,
    kinect_base_position(1) - pickedPoint.y - OBJECT_SIZE_HEIGHT/2,
    kinect_base_position(2) - pickedPoint.z;
  fogActor->GetProperty()->SetColor(233/255.0, 75/255.0, 53/255.0);
  fogActor->SetPosition(cubePosition(0), cubePosition(1), cubePosition(2));
  fogActor->GetProperty()->SetOpacity(0);
  sceneRenderer->AddActor(fogActor);

  

  ret.actor = fogActor;
  ret.pos = cubePosition;
  ret.vel << getRandom(-200, 200)*0.001, -1 - getRandom(0, 300)*0.001, getRandom(-200, 200)*0.001;
  ret.age = 0;
  ret.death = getRandom(40, 50);
  int numCyclesFadeOut = getRandom(4, 10);
  ret.opacityRamp = ClippedRamp(ret.death, 0.0, ret.death - numCyclesFadeOut, 1.0, 0.0, 1.0);

  return ret;
}

int PCLViewer::getRandom(int min, int max) {
  int range = max - min;
  int number = range*2;
  return qrand() % number - (range/2);
}

void PCLViewer::connectSerial(bool on) {
  if(on)
  {
    countDownTimer = 5999;
    bool found = false;
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
      //qDebug() << "Name : " << info.portName();
      //qDebug() << "Description : " << info.description();
      //qDebug() << "Manufacturer: " << info.manufacturer();

      // Example use QSerialPort

      if(QString::compare(info.systemLocation(), ui->txt_connect->text(), Qt::CaseSensitive) == 0)
      {
        found = true;
        serial.setPort(info);
        if (!serial.open(QIODevice::ReadWrite))
          ui->pushButton_connect->setChecked(false);
        else
        {
          serial.setBaudRate(QSerialPort::Baud9600);
          serial.setDataBits(QSerialPort::Data8);
          serial.setParity(QSerialPort::NoParity);
          serial.setStopBits(QSerialPort::OneStop);
          serial.setFlowControl(QSerialPort::NoFlowControl);
        }
      }
    }
    if(!found)
      ui->pushButton_connect->setChecked(false);
    else
    {
      pid.reset();
      applyPID = false;
    }
  }else{
    serial.close();
  }
}

vtkActor* PCLViewer::createText(QString text, QColor color){
      vtkActor* actor = vtkActor::New();
      vtkSmartPointer<vtkVectorText> txt = vtkSmartPointer<vtkVectorText>::New();
      txt->SetText(text.toStdString().c_str());
      vtkSmartPointer<vtkPolyDataMapper> txtRobotMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      txtRobotMapper->SetInput(txt->GetOutput());
      actor->SetMapper(txtRobotMapper);
      actor->GetProperty()->SetColor(color.red()/255.0,color.green()/255.0,color.blue()/255.0);
      actor->GetProperty()->SetAmbient(1.0);
      actor->SetOrientation(0,0,90);
      return actor;
  }