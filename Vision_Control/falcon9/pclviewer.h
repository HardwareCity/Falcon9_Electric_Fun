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

public Q_SLOTS:
  void
  randomButtonPressed ();

  void
  RGBsliderReleased ();

  void
  pSliderValueChanged (int value);

  void
  redSliderValueChanged (int value);

  void
  greenSliderValueChanged (int value);

  void
  blueSliderValueChanged (int value);

  void
  update_cloud();

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr cloud;

  unsigned int red;
  unsigned int green;
  unsigned int blue;

private:
  Ui::PCLViewer *ui;

  QTimer* Update_timer;


  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_new;
  K2G* k2g;
};

#endif // PCLVIEWER_H
