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

#include "QtKittiVisualizer.h"
#include "ui_QtKittiVisualizer.h"

#include <string>

#include <QCheckBox>
#include <QLabel>
#include <QMainWindow>
#include <QSlider>
#include <QWidget>
#include <QFileDialog>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <eigen3/Eigen/Core>

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "KittiFileSystem.h"
#include "KittiDataset.h"

#include "kitti-devkit-raw/tracklets.h"

KittiVisualizerQt::KittiVisualizerQt(QWidget *parent, int argc, char **argv) :
        QMainWindow(parent),
        ui(new Ui::KittiVisualizerQt),
        pcd_opened_(false),
        dataset_index(0),
        dataset(NULL),
        frame_index(0),
        tracklet_index(0),
        pclVisualizer(new pcl::visualization::PCLVisualizer("PCL Visualizer", false)),
        pointCloudVisible(true),
        trackletBoundingBoxesVisible(true),
        trackletPointsVisible(true),
        trackletInCenterVisible(true)
{
    int invalidOptions = parseCommandLineOptions(argc, argv);
    if (invalidOptions) {
        exit(invalidOptions);
    }

    cur_pcd_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    // Set up user interface
    ui->setupUi(this);
    //set up point cloud viewer
    ui->qvtkWidget_pclViewer->SetRenderWindow(pclVisualizer->getRenderWindow());
    pclVisualizer->setupInteractor(ui->qvtkWidget_pclViewer->GetInteractor(),
                                   ui->qvtkWidget_pclViewer->GetRenderWindow());
    pclVisualizer->setBackgroundColor(0, 0, 0);
    pclVisualizer->addCoordinateSystem(1.0);
    pclVisualizer->registerKeyboardCallback(&KittiVisualizerQt::keyboardEventOccurred, *this, 0);
    this->setWindowTitle("Qt KITTI Visualizer");
    ui->qvtkWidget_pclViewer->update();

    ui->slider_frame->setEnabled(false);
    ui->slider_tracklet->setEnabled(false);

    // Connect signals and slots
    connect(ui->btn_openDataset, SIGNAL(clicked()), this, SLOT(openDataset()));
    connect(ui->slider_frame, SIGNAL(valueChanged(int)), this, SLOT(newFrameRequested(int)));
    connect(ui->slider_tracklet, SIGNAL(valueChanged(int)), this, SLOT(newTrackletRequested(int)));
    connect(ui->checkBox_showFramePointCloud, SIGNAL(toggled(bool)), this, SLOT(showFramePointCloudToggled(bool)));
    connect(ui->checkBox_showTrackletBoundingBoxes, SIGNAL(toggled(bool)), this, SLOT(showTrackletBoundingBoxesToggled(bool)));
    connect(ui->checkBox_showTrackletPointClouds, SIGNAL(toggled(bool)), this,
            SLOT(showTrackletPointCloudsToggled(bool)));
    connect(ui->checkBox_showTrackletInCenter, SIGNAL(toggled(bool)), this, SLOT(showTrackletInCenterToggled(bool)));

    connect(ui->btn_openPCD, SIGNAL(clicked()), this, SLOT(openPCD()));
    connect(ui->btn_previous, SIGNAL(clicked()), this, SLOT(previousPCD()));
    connect(ui->btn_next, SIGNAL(clicked()), this, SLOT(nextPCD()));
    connect(ui->cbb_pcdFileList, SIGNAL(activated(const QString&)), this, SLOT(pcdChanged(const QString &)));
}

KittiVisualizerQt::~KittiVisualizerQt()
{
    delete dataset;
    delete ui;
}

int KittiVisualizerQt::parseCommandLineOptions(int argc, char **argv)
{
    // Declare the supported options.
    boost::program_options::options_description desc("Program options");
    desc.add_options()
            ("help", "Produce this help message.")
            ("dataset", boost::program_options::value<int>(), "Set the number of the KITTI data set to be used.");

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    /*if (vm.count("dataset")) {
        dataset_index = vm["dataset"].as<int>();
        std::cout << "Using data set " << dataset_index << "." << std::endl;
        dataset_index = KittiConfig::getDatasetIndex(dataset_index);
    } else {
        dataset_index = 0;
        std::cout << "Data set was not specified." << std::endl;
        std::cout << "Using data set " << KittiConfig::getDatasetNumber(dataset_index) << "." << std::endl;
    }*/
    return 0;
}

void KittiVisualizerQt::openDataset()
{
    std::string dataset_dir = QFileDialog::getExistingDirectory(this, tr("Open KiTTI Sequence Directory"), "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks).toStdString();
    if (dataset_dir.empty()) {
        return;
    }

    // clear pre-load dataset
    if (dataset) {
        resetUI();
        delete dataset;
    }

    dataset = new KittiDataset(dataset_dir);
    if (!loadPointCloud()) {
        return;
    }
    pcd_opened_ = false;
    if (pointCloudVisible) {
        showPointCloud();
    }
    loadAvailableTracklets();
    if (trackletBoundingBoxesVisible) {
        showTrackletBoxes();
    }
    loadTrackletPoints();
    if (trackletPointsVisible) {
        showTrackletPoints();
    }
    if (trackletInCenterVisible) {
        showTrackletInCenter();
    }

    ui->slider_frame->setEnabled(true);
    ui->slider_tracklet->setEnabled(true);

    ui->slider_frame->setRange(0, dataset->getNumberOfFrames() - 1);
    ui->slider_frame->setValue(frame_index);
    if (availableTracklets.size() != 0) {
        ui->slider_tracklet->setRange(0, availableTracklets.size() - 1);
    }
    else {
        ui->slider_tracklet->setRange(0, 0);
    }
    ui->slider_tracklet->setValue(tracklet_index);

    updateFrameLabel();
    updateTrackletLabel();
    ui->qvtkWidget_pclViewer->update();
}

void KittiVisualizerQt::openPCD()
{
    // clear pre-open
    clearPCD();
    hidePCD();

    pcd_dir_ = QFileDialog::getExistingDirectory(this, tr("Open pcd Directory"), "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks).toStdString();
    if (pcd_dir_.empty()) {
        return;
    }

    loadPCD(pcd_dir_ + "/", pcd_filelists_, ".pcd");
    if (pcd_filelists_.empty()) {
        std::cerr << "Error in opening pcd: No pcd files founded!!" << std::endl;
        return;
    }

    pcd_opened_ = true;
    QString tmp_qstring;
    for (size_t i = 0u; i < pcd_filelists_.size(); ++i) {
        ui->cbb_pcdFileList->addItem(tmp_qstring.fromStdString(pcd_filelists_[i]));
    }

    cur_pcd_index_ = 0;
    showPCD();
    ui->qvtkWidget_pclViewer->update();
}

bool KittiVisualizerQt::loadNextFrame()
{
    newFrameRequested(frame_index + 1);
}

bool KittiVisualizerQt::loadPreviousFrame()
{
    newFrameRequested(frame_index - 1);
}

void KittiVisualizerQt::getTrackletColor(const KittiTracklet& tracklet, int& r, int& g, int& b)
{
    KittiDataset::getColor(tracklet.objectType.c_str(), r, g, b);
}

void KittiVisualizerQt::showFramePointCloudToggled(bool value)
{
    pointCloudVisible = value;
    if (pointCloudVisible) {
        showPointCloud();
    } else {
        hidePointCloud();
    }
    ui->qvtkWidget_pclViewer->update();
}

void KittiVisualizerQt::newFrameRequested(int value)
{
    if (frame_index == value)
        return;

    resetUI();

    frame_index = value;
    if (frame_index >= dataset->getNumberOfFrames())
        frame_index = dataset->getNumberOfFrames() - 1;
    if (frame_index < 0)
        frame_index = 0;

    loadPointCloud();
    if (pointCloudVisible)
        showPointCloud();
    loadAvailableTracklets();
    if (trackletBoundingBoxesVisible)
        showTrackletBoxes();
    loadTrackletPoints();
    if (trackletPointsVisible)
        showTrackletPoints();
    if (tracklet_index >= availableTracklets.size())
        tracklet_index = availableTracklets.size() - 1;
    if (tracklet_index < 0)
        tracklet_index = 0;
    if (trackletInCenterVisible)
        showTrackletInCenter();

    if (availableTracklets.size() != 0)
        ui->slider_tracklet->setRange(0, availableTracklets.size() - 1);
    else
        ui->slider_tracklet->setRange(0, 0);
    ui->slider_tracklet->setValue(tracklet_index);

    updateFrameLabel();
    updateTrackletLabel();
    ui->qvtkWidget_pclViewer->update();
}

void KittiVisualizerQt::newTrackletRequested(int value)
{
    if (tracklet_index == value)
        return;

    if (trackletInCenterVisible)
        hideTrackletInCenter();

    tracklet_index = value;
    if (tracklet_index >= availableTracklets.size())
        tracklet_index = availableTracklets.size() - 1;
    if (tracklet_index < 0)
        tracklet_index = 0;
    if (trackletInCenterVisible)
        showTrackletInCenter();

    updateTrackletLabel();
    ui->qvtkWidget_pclViewer->update();
}

void KittiVisualizerQt::showTrackletBoundingBoxesToggled(bool value)
{
    trackletBoundingBoxesVisible = value;
    if (trackletBoundingBoxesVisible) {
        showTrackletBoxes();
    } else {
        hideTrackletBoxes();
    }
    ui->qvtkWidget_pclViewer->update();
}

void KittiVisualizerQt::showTrackletPointCloudsToggled(bool value)
{
    trackletPointsVisible = value;
    if (trackletPointsVisible) {
        showTrackletPoints();
    } else {
        hideTrackletPoints();
    }
    ui->qvtkWidget_pclViewer->update();
}

void KittiVisualizerQt::showTrackletInCenterToggled(bool value)
{
    trackletInCenterVisible = value;
    if (trackletInCenterVisible) {
        showTrackletInCenter();
    } else {
        hideTrackletInCenter();
    }
    ui->qvtkWidget_pclViewer->update();
}

bool KittiVisualizerQt::loadPointCloud()
{
    pointCloud = dataset->getPointCloud(frame_index);
    return (pointCloud->size() > 0);
}

void KittiVisualizerQt::showPointCloud()
{
    pcl::visualization::PointCloudColorHandlerGenericField<KittiPoint> colorHandler(pointCloud, "intensity");
    pclVisualizer->addPointCloud<KittiPoint>(pointCloud, colorHandler, "point_cloud");
}

void KittiVisualizerQt::hidePointCloud()
{
    pclVisualizer->removePointCloud("point_cloud");
}

void KittiVisualizerQt::loadAvailableTracklets()
{
    Tracklets& tracklets = dataset->getTracklets();
    int tracklet_id;
    int number_of_tracklets = tracklets.numberOfTracklets();
    for (tracklet_id = 0; tracklet_id < number_of_tracklets; tracklet_id++) {
        KittiTracklet* tracklet = tracklets.getTracklet(tracklet_id);
        if (tracklet->first_frame <= frame_index && tracklet->lastFrame() >= frame_index) {
            availableTracklets.push_back(*tracklet);
        }
    }
}

void KittiVisualizerQt::clearAvailableTracklets()
{
    availableTracklets.clear();
}

void KittiVisualizerQt::updateFrameLabel()
{
    std::stringstream text;
    text << "Frame: "
         << frame_index + 1 << " of " << dataset->getNumberOfFrames()
         << std::endl;
    ui->label_frame->setText(text.str().c_str());

}

void KittiVisualizerQt::updateTrackletLabel()
{
    if (availableTracklets.size()) {
        std::stringstream text;
        KittiTracklet tracklet = availableTracklets.at(tracklet_index);
        text << "Tracklet: "
             << tracklet_index + 1 << " of " << availableTracklets.size()
             << " (\"" << tracklet.objectType
             << "\", " << croppedTrackletPointClouds.at(tracklet_index).get()->size()
             << " points)"
             << std::endl;
        ui->label_tracklet->setText(text.str().c_str());
    } else {
        std::stringstream text;
        text << "Tracklet: "
             << "0 of 0"
             << std::endl;
        ui->label_tracklet->setText(text.str().c_str());
    }
}

void KittiVisualizerQt::showTrackletBoxes()
{
    double boxHeight = 0.0d;
    double boxWidth = 0.0d;
    double boxLength = 0.0d;
    int pose_number = 0;

    for (int i = 0; i < availableTracklets.size(); ++i) {
        // Create the bounding box
        const KittiTracklet& tracklet = availableTracklets.at(i);
        int r, g, b;
        getTrackletColor(tracklet, r, g, b);

        boxHeight = tracklet.h;
        boxWidth = tracklet.w;
        boxLength = tracklet.l;
        pose_number = frame_index - tracklet.first_frame;
        const Tracklets::tPose& tpose = tracklet.poses.at(pose_number);
        Eigen::Vector3f boxTranslation;
        boxTranslation[0] = (float) tpose.tx;
        boxTranslation[1] = (float) tpose.ty;
        boxTranslation[2] = (float) tpose.tz + (float) boxHeight / 2.0f;
        Eigen::Quaternionf boxRotation = Eigen::Quaternionf(
                Eigen::AngleAxisf((float) tpose.rz, Eigen::Vector3f::UnitZ()));

        // Add the bounding box to the visualizer
        std::string viewer_id = "tracklet_box_" + i;
        pclVisualizer->addCube(boxTranslation, boxRotation, boxLength, boxWidth, boxHeight, viewer_id);
    }
}

void KittiVisualizerQt::hideTrackletBoxes()
{
    for (int i = 0; i < availableTracklets.size(); ++i) {
        std::string viewer_id = "tracklet_box_" + i;
        pclVisualizer->removeShape(viewer_id);
    }
}

void KittiVisualizerQt::loadTrackletPoints()
{
    for (int i = 0; i < availableTracklets.size(); ++i) {
        // Create the tracklet point cloud
        const KittiTracklet& tracklet = availableTracklets.at(i);
        pcl::PointCloud<KittiPoint>::Ptr trackletPointCloud = dataset->getTrackletPointCloud(pointCloud, tracklet, frame_index);
        pcl::PointCloud<KittiPoint>::Ptr trackletPointCloudTransformed(new pcl::PointCloud<KittiPoint>);

        Eigen::Vector3f transformOffset;
        transformOffset[0] = 0.0f;
        transformOffset[1] = 0.0f;
        transformOffset[2] = 6.0f;
        pcl::transformPointCloud(*trackletPointCloud, *trackletPointCloudTransformed, transformOffset, Eigen::Quaternionf::Identity());

        // Store the tracklet point cloud
        //croppedTrackletPointClouds.push_back(trackletPointCloudTransformed);
        croppedTrackletPointClouds.push_back(trackletPointCloud);
    }
}

void KittiVisualizerQt::showTrackletPoints()
{
    for (int i = 0; i < availableTracklets.size(); ++i) {
        // Create a color handler for the tracklet point cloud
        const KittiTracklet& tracklet = availableTracklets.at(i);
        int r, g, b;
        getTrackletColor(tracklet, r, g, b);
        pcl::visualization::PointCloudColorHandlerCustom<KittiPoint> colorHandler(croppedTrackletPointClouds.at(i), r, g, b);

        // Add tracklet point cloud to the visualizer
        std::string viewer_id = "cropped_tracklet_" + i;
        pclVisualizer->addPointCloud<KittiPoint>(croppedTrackletPointClouds.at(i), colorHandler, viewer_id);
    }
}

void KittiVisualizerQt::hideTrackletPoints()
{
    for (int i = 0; i < availableTracklets.size(); ++i) {
        std::string viewer_id = "cropped_tracklet_" + i;
        pclVisualizer->removeShape(viewer_id);
    }
}

void KittiVisualizerQt::clearTrackletPoints()
{
    croppedTrackletPointClouds.clear();
}

void KittiVisualizerQt::showTrackletInCenter()
{
    if (availableTracklets.size()) {
        // Create the centered tracklet point cloud
        const KittiTracklet& tracklet = availableTracklets.at(tracklet_index);
        pcl::PointCloud<KittiPoint>::Ptr cloudOut = dataset->getTrackletPointCloud(pointCloud, tracklet, frame_index);
        pcl::PointCloud<KittiPoint>::Ptr cloudOutTransformed(new pcl::PointCloud<KittiPoint>);

        int pose_number = frame_index - tracklet.first_frame;
        Tracklets::tPose tpose = tracklet.poses.at(pose_number);

        Eigen::Vector3f transformOffset((float) -(tpose.tx), (float) -(tpose.ty),
                                        (float) -(tpose.tz + tracklet.h / 2.0));

        Eigen::AngleAxisf angleAxisZ(-tpose.rz, Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf transformRotation(angleAxisZ);
        pcl::transformPointCloud(*cloudOut, *cloudOutTransformed, transformOffset, Eigen::Quaternionf::Identity());
        pcl::transformPointCloud(*cloudOutTransformed, *cloudOut, Eigen::Vector3f::Zero(), transformRotation);

        // Create color handler for the centered tracklet point cloud
        pcl::visualization::PointCloudColorHandlerCustom<KittiPoint> colorHandler(cloudOut, 0, 255, 0);

        // Add the centered tracklet point cloud to the visualizer
        pclVisualizer->addPointCloud<KittiPoint>(cloudOut, colorHandler, "centered_tracklet");
    }
}

void KittiVisualizerQt::hideTrackletInCenter()
{
    if (availableTracklets.size()) {
        pclVisualizer->removeShape("centered_tracklet");
    }
}

void KittiVisualizerQt::setFrameNumber(int frameNumber)
{
    frame_index = frameNumber;
}

void KittiVisualizerQt::keyboardEventOccurred(
        const pcl::visualization::KeyboardEvent& event, void *viewer_void)
{
    if (event.getKeyCode() == 0 && event.keyDown()) {
        if (event.getKeySym() == "Left") {
            loadPreviousFrame();
        } else if (event.getKeySym() == "Right") {
            loadNextFrame();
        }
    }
}

void KittiVisualizerQt::resetUI()
{
    if (trackletInCenterVisible) {
        hideTrackletInCenter();
    }
    if (trackletPointsVisible) {
        hideTrackletPoints();
    }
    clearTrackletPoints();
    if (trackletBoundingBoxesVisible) {
        hideTrackletBoxes();
    }
    clearAvailableTracklets();
    if (pointCloudVisible) {
        hidePointCloud();
    }
}

void KittiVisualizerQt::loadPCD(const std::string& dir_path,
                                std::vector<std::string>& out_filelists,
                                std::string type)
{
    struct dirent* ptr;
    DIR* dir;
    dir = opendir(dir_path.c_str());
    out_filelists.clear();
    while ((ptr = readdir(dir)) != NULL) {
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.') {
            continue;
        }
        if (type.size() <= 0) {
            out_filelists.push_back(ptr->d_name);
        } else {
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(), type.size());
            if (tmp_cut_type == type) {
                out_filelists.push_back(ptr->d_name);
            }
        }
    }

    sortFilelists(out_filelists, type);
}

void KittiVisualizerQt::nextPCD()
{
    hidePCD();
    cur_pcd_index_++;
    if (cur_pcd_index_ >= pcd_filelists_.size()) {
        cur_pcd_index_ = pcd_filelists_.size() - 1;
    }

    QString tmp_qstring;
    ui->cbb_pcdFileList->setCurrentText(tmp_qstring.fromStdString(pcd_filelists_[cur_pcd_index_]));

    showPCD();
    ui->qvtkWidget_pclViewer->update();
}

void KittiVisualizerQt::previousPCD()
{
    hidePCD();
    cur_pcd_index_--;
    if (cur_pcd_index_ < 0) {
        cur_pcd_index_ = 0;
    }

    QString tmp_qstring;
    ui->cbb_pcdFileList->setCurrentText(tmp_qstring.fromStdString(pcd_filelists_[cur_pcd_index_]));

    showPCD();
    ui->qvtkWidget_pclViewer->update();
}

void KittiVisualizerQt::showPCD()
{
    std::string tmp_pcd_file_path = pcd_dir_ + "/" + pcd_filelists_[cur_pcd_index_];
    if (pcl::io::loadPCDFile<PcdPoint>(tmp_pcd_file_path.c_str(), *cur_pcd_ptr_) == -1) {
        std::cerr << "Error in loading " + pcd_filelists_[cur_pcd_index_] + " !!"
                  << std::endl;
        return;
    }

    pcl::visualization::PointCloudColorHandlerGenericField<PcdPoint> colorHandler(cur_pcd_ptr_, "intensity");
    pclVisualizer->addPointCloud<PcdPoint>(cur_pcd_ptr_, colorHandler, "point_cloud");
}

void KittiVisualizerQt::hidePCD()
{
    pclVisualizer->removeShape("point_cloud");
}

void KittiVisualizerQt::clearPCD()
{
    pcd_filelists_.clear();
    ui->cbb_pcdFileList->clear();
}

bool computePairNum(std::pair<double, std::string> pair1,
                    std::pair<double, std::string> pair2)
{
    return pair1.first < pair2.first;
}
void KittiVisualizerQt::sortFilelists(std::vector<std::string>& filists,
                                      std::string type)
{
    if (filists.empty()) {
        return;
    }
    std::vector<std::pair<double, std::string> > filelists_pair;
    for (size_t i = 0u; i < filists.size(); ++i) {
        std::string tmp_string = filists[i];
        std::string tmp_num_string = tmp_string.substr(0, tmp_string.size() - type.size());
        double tmp_num = atof(tmp_num_string.c_str());
        std::pair<double, std::string> tmp_pair;
        tmp_pair.first = tmp_num;
        tmp_pair.second = tmp_string;
        filelists_pair.push_back(tmp_pair);
    }
    std::sort(filelists_pair.begin(), filelists_pair.end(), computePairNum);
    filists.clear();
    for (int i = 0; i < filelists_pair.size(); ++i) {
        filists.push_back(filelists_pair[i].second);
    }
}

void KittiVisualizerQt::pcdChanged(const QString& cur_pcd)
{
    hidePCD();
    std::string tmp_pcd_file_name = cur_pcd.toStdString();
    int tmp_index = findIndex(pcd_filelists_, tmp_pcd_file_name);
    if (tmp_index == -1) {
        std::cerr << "error!! no " << tmp_pcd_file_name << " !"
                  << std::endl;
        return;
    }
    cur_pcd_index_ = tmp_index;
    showPCD();
    ui->qvtkWidget_pclViewer->update();
}

int KittiVisualizerQt::findIndex(const std::vector<std::string>& file_lists,
                                 const std::string& cur_file_name)
{
    int tmp_index = -1;
    if (file_lists.empty()) {
        return tmp_index;
    }
    for (int i = 0; i < file_lists.size(); ++i) {
        if (file_lists[i] == cur_file_name) {
            tmp_index = i;
            return tmp_index;
        }
    }
    return tmp_index;
}
