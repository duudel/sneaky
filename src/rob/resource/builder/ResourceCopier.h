
#ifndef H_ROB_RESOURCE_COPIER_H
#define H_ROB_RESOURCE_COPIER_H

#include "ResourceBuilder.h"

namespace rob
{

    class ResourceCopier : public ResourceBuilder
    {
    public:
        ResourceCopier();
        bool Build(const std::string &directory, const std::string &filename,
                   const std::string &destDirectory, const std::string &destFilename) override;
    };

} // rob

#endif // H_ROB_RESOURCE_COPIER_H

