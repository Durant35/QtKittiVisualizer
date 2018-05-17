/*

Copyright 2016 Mark Muth

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/

#ifndef QT_KITTI_VISUALIZER_H
#define QT_KITTI_VISUALIZER_H

#include <vector>
// Qt
#include <QMainWindow>
#include <QWidget>

// Boost
#include <boost/program_options.hpp>

// PCL
#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/pcl_visualizer.h>

// VTK
#include <vtkRenderWindow.h>

#include "KittiDataset.h"

#include <kitti-devkit-raw/tracklets.h>

namespace Ui {
    class KittiVisualizerQt;
}

typedef pcl::PointXYZI PcdPoint;

class KittiVisualizerQt : public QMainWindow {
Q_OBJECT

public:
    KittiVisualizerQt(QWidget *parent, int argc, char **argv);

    virtual ~KittiVisualizerQt();

    bool loadDataset();

    bool loadNextFrame();

    bool loadPreviousFrame();

    void getTrackletColor(const KittiTracklet& tracklet, int& r, int& g, int& b);

public slots:
    void openDataset();
    void openPCD();

    void newFrameRequested(int value);
    void newTrackletRequested(int value);

    void showFramePointCloudToggled(bool value);
    void showTrackletBoundingBoxesToggled(bool value);
    void showTrackletPointCloudsToggled(bool value);
    void showTrackletInCenterToggled(bool value);

    void nextPCD();
    void previousPCD();
    void pcdChanged(const QString& cur_pcd);

private:
    int parseCommandLineOptions(int argc, char **argv);

    int dataset_index;
    KittiDataset* dataset;

    int frame_index;

    int tracklet_index;

    pcl::visualization::PCLVisualizer::Ptr pclVisualizer;

    void loadAvailableTracklets();

    void clearAvailableTracklets();

    std::vector<KittiTracklet> availableTracklets;

    void updateFrameLabel();
    void updateTrackletLabel();

    bool loadPointCloud();
    void showPointCloud();
    void hidePointCloud();

    bool pointCloudVisible;
    KittiPointCloud::Ptr pointCloud;

    void showTrackletBoxes();
    void hideTrackletBoxes();

    bool trackletBoundingBoxesVisible;

    void loadTrackletPoints();
    void showTrackletPoints();
    void hideTrackletPoints();
    void clearTrackletPoints();

    bool trackletPointsVisible;
    std::vector<KittiPointCloud::Ptr> croppedTrackletPointClouds;

    void showTrackletInCenter();
    void hideTrackletInCenter();

    bool trackletInCenterVisible;

    void setFrameNumber(int frameNumber);

    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
                               void* viewer_void);
    void resetUI();

    bool pcd_opened_;
    std::string pcd_dir_;
    std::vector<std::string> pcd_filelists_;
    int cur_pcd_index_;
    pcl::PointCloud<PcdPoint>::Ptr cur_pcd_ptr_;

    void loadPCD(const std::string& dir_path, std::vector<std::string>& out_filelists, std::string type);
    void showPCD();
    void hidePCD();
    void clearPCD();
    void sortFilelists(std::vector<std::string>& filists, std::string type);
    int findIndex(const std::vector<std::string>& file_lists, const std::string& cur_file_name);

    Ui::KittiVisualizerQt* ui;
};

#endif // QT_KITTI_VISUALIZER_H
