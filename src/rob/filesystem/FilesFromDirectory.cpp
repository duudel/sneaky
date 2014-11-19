
#include "FilesFromDirectory.h"
#include "FileStat.h"

#include <cstring>
#include <dirent.h>

namespace rob
{

    static void GetFilesFromDirectoryImpl(const std::string &directory,
                                         const std::string &subDir,
                                         std::vector<std::string> &files,
                                         bool recursive)
    {
        DIR *dir = nullptr;
        struct dirent *entry = nullptr;

        dir = ::opendir(directory.c_str());
        if (dir)
        {
            while((entry = ::readdir(dir)))
            {
                const std::string filepath = directory + entry->d_name;
                const std::string file = subDir + entry->d_name;

                if (std::strcmp(entry->d_name, "..") != 0
                    && std::strcmp(entry->d_name, ".") != 0)
                {
                    if (IsDirectory(filepath.c_str()))
                    {
                        if (recursive)
                            GetFilesFromDirectoryImpl(filepath, file + "/", files, true);
                    }
                    else
                    {
                        files.push_back(file);
                    }
                }
            }
            ::closedir(dir);
        }
        else
        {
//            log::warning("Failed to open directory '%'.", directory.c_str());
        }
    }

    void GetFilesFromDirectory(const std::string &dir, std::vector<std::string> &files, bool recursive)
    {
        GetFilesFromDirectoryImpl(dir, "", files, recursive);
    }

} // rob
