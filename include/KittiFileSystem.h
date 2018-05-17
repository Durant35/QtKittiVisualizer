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

#ifndef KITTIFILESYSTEM_H
#define KITTIFILESYSTEM_H

#include <string>
#include <vector>

#include <boost/filesystem/path.hpp>

/**
 * @brief The KittiConfig class
 *
 * Define where the KITTI data sets are stored. Templated strings provide for a
 * high flexibility in storing and accessing your files.
 *
 * The predefined values assume the following filesystem hierarchy:
 *
 * /QtKittiVisualizer (source)
 * /QtKittiVisualizer-build (working directory)
 * /KittiData
 *   /raw
 *     /%|04|_sync (datasets, e.g. 0001_sync)
 *       /velodyne_points
 *         /data
 *           /%|010|.bin (point clouds, e.g. 0000000000.bin)
 *       /tracklet_labels.xml (tracklets)
 *
 * You can change the predefined values to your needs in KittiConfig.cpp.
 */
class KittiFileSystem {
public:
    static boost::filesystem::path getPointCloudDir(std::string dataset);
    static boost::filesystem::path getPointCloudPath(std::string dataset, int frameId);

    static boost::filesystem::path getTrackletsPath(std::string dataset);

private:
    // Following variables describe the filesystem hierarchy of the KITTI dataset
    static std::string point_cloud_directory;
    static std::string point_cloud_file_template;
    static std::string tracklets_file_name;
};

#endif // KITTIFILESYSTEM_H
