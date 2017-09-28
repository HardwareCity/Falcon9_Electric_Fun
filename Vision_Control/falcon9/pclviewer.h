#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>

// Kinect
#include "k2g.h"

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

#include <Eigen/Dense>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

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

  pcl::PointXYZ pickedPoint;
  bool picked_event;

public Q_SLOTS:
  void
  randomButtonPressed ();

  void
  RGBsliderReleased ();

  void
  pSliderValueChanged (int value);

  void
  minSliderValueChanged (int value);

  void
  maxSliderValueChanged (int value);

  void
  blueSliderValueChanged (int value);

  void
  update_cloud();


protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
  PointCloudT::Ptr cloud;

  unsigned int minDist;
  unsigned int maxDist;
  unsigned int blue;

private:
  Ui::PCLViewer *ui;




  QTimer* Update_timer;
  Eigen::Vector4f kinect_base_position;

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_new;
  K2G* k2g;
};

#endif // PCLVIEWER_H
