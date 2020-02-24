/*
 * @Description: 一些文件读写的方法
 * @Author: Ren Qian
 * @Date: 2020-02-24 20:09:32
 */
#include "lidar_localization/tools/file_manager.hpp"

#include <boost/filesystem.hpp>
#include "glog/logging.h"

namespace lidar_localization {
bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.open(file_path.c_str(), std::ios::app);
    if (!ofs) {
        LOG(WARNING) << "无法生成文件: " << file_path;
        return false;
    }

    return true;
}

bool FileManager::CreateDirectory(std::string directory_path) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }
    if (!boost::filesystem::is_directory(directory_path)) {
        LOG(WARNING) << "无法建立文件夹: " << directory_path;
        return false;
    }
    return true;
}
}