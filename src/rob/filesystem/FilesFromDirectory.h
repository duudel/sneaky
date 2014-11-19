
#ifndef H_FILES_FROM_DIRECTORY_H
#define H_FILES_FROM_DIRECTORY_H

#include <string>
#include <vector>

namespace rob
{

    void GetFilesFromDirectory(const std::string &dir, std::vector<std::string> &files, bool recursive);

} // rob

#endif // H_FILES_FROM_DIRECTORY_H

