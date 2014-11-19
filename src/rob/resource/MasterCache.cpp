
#include "MasterCache.h"
#include "../filesystem/FilesFromDirectory.h"
#include "../Log.h"

namespace rob
{

    MasterCache::MasterCache(Graphics *graphics, AudioSystem *audio, LinearAllocator &alloc)
        : m_textures(graphics)
        , m_sounds(audio)
        , m_fonts(graphics, this)
        , m_resources()
    {
        Scan("data/");
    }

    void MasterCache::Scan(const char * const dir)
    {
        const std::string directory(dir);
        std::vector<std::string> files;
        GetFilesFromDirectory(directory, files, true);

        for (const std::string &file : files)
        {
            ResourceID id(file.c_str());
            Resource resource;
            const std::string filepath = directory + file;
            const size_t len = filepath.length();
            if (len < sizeof(resource.m_filepath))
            {
                for (size_t i = 0; i < len; i++)
                    resource.m_filepath[i] = filepath[i];
                resource.m_filepath[len] = 0;
            }
            else
            {
                log::Error("MasterCache: Filepath too long: ", filepath.c_str());
                continue;
            }
            m_resources[id] = resource;
            log::Debug("MasterCache: Found: ", filepath.c_str(), ": ", uint32_t(id));
        }
    }

    TextureHandle MasterCache::GetTexture(ResourceID id)
    {
        const Resource *resource = nullptr;
        if (FindResource(id, &resource))
            return m_textures.Get(id, resource->m_filepath);
        ReportInvalidResource(id);
        return InvalidHandle;
    }

    SoundHandle MasterCache::GetSound(ResourceID id)
    {
        const Resource *resource = nullptr;
        if (FindResource(id, &resource))
            return m_sounds.Get(id, resource->m_filepath);
        ReportInvalidResource(id);
        return InvalidSound;
    }

    Font MasterCache::GetFont(ResourceID id)
    {
        const Resource *resource = nullptr;
        if (FindResource(id, &resource))
            return m_fonts.Get(id, resource->m_filepath);
        ReportInvalidResource(id);
        return Font();
    }

    bool MasterCache::FindResource(ResourceID id, const Resource **resource) const
    {
        auto it = m_resources.find(id);
        if (it == m_resources.end())
            return false;
        *resource = &it->second;
        return true;
    }

    void MasterCache::ReportInvalidResource(ResourceID id) const
    {
        log::Info("MasterCache: Invalid reosource id: ", uint32_t(id));
    }

} // rob
