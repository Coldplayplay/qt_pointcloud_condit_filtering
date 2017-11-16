#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>
#include <string>
// Qt
#include <QMainWindow>

#include "build/ui_pclviewer.h"

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
using namespace std;
using pcl::visualization::PointCloudColorHandlerCustom;

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
 

  //viewPair(cloudin,cloudout,range_cond);

public Q_SLOTS:

  void updateLabelValue1(int value);
  void updateLabelValue2(int value);
  void updateLabelValue3(int value);
  void updateLabelValue4(int value);
  void updateLabelValue5(int value);
  void updateLabelValue6(int value); 


  void  saveButtonPressed ();  

  void  x1SliderValueChanged (int value);
  void  x2SliderValueChanged (int value);
  void  y1SliderValueChanged (int value);
  void  y2SliderValueChanged (int value);
  void  z1SliderValueChanged (int value);
  void  z2SliderValueChanged (int value);

  void  conditFilter();

  //void  voxelFilter(cloudin,cloudout,leafsize);

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;  
  int v1, v2;
  PointCloudT::Ptr cloudin;
  PointCloudT::Ptr cloudout;

  double x1;
  double x2;
  double y1;
  double y2;
  double z1;
  double z2;
//unsigned double leafsize;

private:
  Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H
