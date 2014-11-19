
#ifndef H_ROB_TEXTURE_BUILDER_H
#define H_ROB_TEXTURE_BUILDER_H

#include "ResourceBuilder.h"

namespace rob
{

    class TextureBuilder : public ResourceBuilder
    {
    public:
        TextureBuilder();
        bool Build(const std::string &directory, const std::string &filename,
                   const std::string &destDirectory, const std::string &destFilename) override;
    };

} // rob

#endif // H_ROB_TEXTURE_BUILDER_H

