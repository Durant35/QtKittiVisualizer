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

#include "KittiFileSystem.h"

#include <string>
#include <vector>
#include <iostream>

#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>

std::string KittiFileSystem::point_cloud_directory = "velodyne_points/data";
std::string KittiFileSystem::point_cloud_file_template = "%|010|.bin";
std::string KittiFileSystem::tracklets_file_name = "tracklet_labels.xml";

boost::filesystem::path KittiFileSystem::getPointCloudDir(std::string dataset)
{
    return boost::filesystem::path(dataset)
           / point_cloud_directory;
}

boost::filesystem::path KittiFileSystem::getPointCloudPath(std::string dataset, int frameId)
{
    return getPointCloudDir(dataset)
           / (boost::format(point_cloud_file_template) % frameId).str();
}

boost::filesystem::path KittiFileSystem::getTrackletsPath(std::string dataset)
{
    return boost::filesystem::path(dataset)
           / tracklets_file_name;
}
