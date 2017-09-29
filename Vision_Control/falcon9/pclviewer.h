#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>

// Kinect
#include "k2g.h"
#include "AbsoluteObjectKF.h"
#include "PID.h"

// Qt
#include <QMainWindow>
#include <QTimer>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h> // filter by distance

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h> // region growing with rgb
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h> // compute 3d centroid

#include <Eigen/Dense>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include "vtkRenderer.h"
#include "vtkImageActor.h"
#include "vtkImageFlip.h"
#include "vtkImageImport.h"
#include "vtkImageData.h"
#include "vtkCamera.h"
#include "vtkCubeSource.h"
#include "vtkSmartPointer.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkMath.h"
#include "vtkSphereSource.h"
#include "vtkMatrix4x4.h"
#include "vtkTransform.h"
#include "vtkPerspectiveTransform.h"
#include "vtkTransform.h"
#include "vtkPlane.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();

  pcl::PointXYZRGB pickedPoint;
  bool picked_event;
  bool run;

public Q_SLOTS:
  void
  saveButtonPressed ();

  void
  RGBsliderReleased ();

  void
  pSliderValueChanged (int value);

  void
  minSliderValueChanged (int value);

  void
  maxSliderValueChanged (int value);

  void
  minHeightSliderValueChanged (int value);

  void kpChanged(double val);
  void kiChanged(double val);
  void kdChanged(double val);
  void maxIntChanged(double val);

  void
  update_cloud();

  void setOrigin();


protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
  PointCloudT::Ptr cloud;

  unsigned int minDist;
  unsigned int maxDist;
  unsigned int minHeight;

private:
  // User Settings
  void loadSettings();
  void saveSettings();
  QString m_sSettingsFile;

  Ui::PCLViewer *ui;

  AbsoluteObjectKF kf;
  PID pid;
  float controlOutput;

  QTimer* Update_timer;
  Eigen::Vector4f kinect_base_position;
  Eigen::Vector3f rocket_origin;
  Eigen::Vector3f rocketPos;

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_new;
  K2G* k2g;

  // AR VTK
  vtkRenderWindow *renWin;
  vtkRenderer *backgroundRenderer;
  vtkRenderer *sceneRenderer;
  vtkImageImport *importer;
  vtkImageActor *imageActor;
  vtkImageFlip *imageFlip;

  vtkActor* cubeActor;
};

#endif // PCLVIEWER_H
