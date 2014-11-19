
#include "ResourceBuilder.h"
#include "../../filesystem/FileStat.h"
#include "../../Log.h"

namespace rob
{

    ResourceBuilder* ResourceBuilder::m_head = nullptr;
    ResourceBuilder* ResourceBuilder::m_tail = nullptr;

    ResourceBuilder::ResourceBuilder()
        : m_extensions()
        , m_newExtension()
        , m_next(nullptr)
    {
        if (m_head)
        {
            m_head->m_next = this;
            m_head = this;
        }
        else
        {
            m_head = this;
            m_tail = this;
        }
    }

    bool ResourceBuilder::TryBuild(const std::string &directory, const std::string &filename,
                                   const std::string &destDirectory, const std::string &destFilename)
    {
        for (size_t i = 0; i < m_extensions.size(); i++)
        {
            const std::string &ext = m_extensions[i];

            const size_t fnlen = filename.length();
            const size_t extlen = ext.length();
            if (fnlen < extlen)
                continue;

            const size_t pos = fnlen - extlen;
            if (filename.compare(pos, extlen, ext) == 0)
            {
                std::string newFilename = destFilename;
                if (!m_newExtension.empty())
                {
                    const size_t pos = destFilename.length() - extlen;
                    newFilename.resize(pos);
                    newFilename += m_newExtension;
                }
                if (!FileExists(newFilename.c_str()) ||
                    GetModifyTime(newFilename.c_str()) < GetModifyTime(filename.c_str()))
                {
                    if (Build(directory, filename, destDirectory, newFilename))
                    {
                        log::Info("Built ", filename.c_str(), " to ", newFilename.c_str());
                    }
                    else
                    {
                        log::Error("Could not build ", filename.c_str(), " to ", newFilename.c_str());
                    }
                }
                return true;
            }
        }
        return false;
    }

} // rob
