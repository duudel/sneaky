
#include "MasterBuilder.h"
#include "../../filesystem/FilesFromDirectory.h"
#include "../../Log.h"

#include "TextureBuilder.h"
#include "ResourceCopier.h"

namespace rob
{

    TextureBuilder  g_textureBuilder;
    ResourceCopier  g_resourceCopier;

    void MasterBuilder::Build(const char * const source, const char * const dest)
    {
        std::vector<std::string> files;
        GetFilesFromDirectory(source, files, true);

        for (const std::string &file : files)
        {
            const std::string sourceFile = std::string(source) + "/" + file;
            const std::string destFile = std::string(dest) + "/" + file;

            ResourceBuilder *builder = ResourceBuilder::m_tail;
            while (builder)
            {
                if (builder->TryBuild(source, sourceFile, dest, destFile))
                    break;
                builder = builder->m_next;
            }
        }
    }

} // rob
