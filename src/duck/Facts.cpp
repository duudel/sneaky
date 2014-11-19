
#include "Facts.h"
#include "rob/memory/LinearAllocator.h"
#include "rob/filesystem/FileStat.h"
#include "rob/filesystem/FileSystem.h"
#include "rob/Assert.h"

namespace sneaky
{

    Facts::Facts()
        : m_facts(nullptr)
        , m_factCount(0)
        , m_lines(nullptr)
    { }

    Facts::~Facts()
    {
        for (size_t i = 0; i < m_factCount; i++)
            m_facts[i].~Fact();
    }

    bool Facts::Load(rob::LinearAllocator &alloc)
    {
        m_facts = alloc.AllocateArray<Fact>(MAX_FACTS);
        m_lines = alloc.AllocateArray<char*>(MAX_LINES);

        const char * const factsPath = "data/facts.txt";
        const size_t fileSize = rob::GetFileSize(factsPath);
        if (fileSize == 0) return false;

        char *data = (char*)alloc.Allocate(fileSize+1);
        const char *const dataEnd = data + fileSize+1;

        rob::fs::File file = rob::fs::OpenToRead(factsPath);
        rob::fs::Read(file, data, fileSize);
        rob::fs::Close(file);

        char *p = data;
        size_t line = 0;
        while (p < dataEnd)
        {
            Fact &f = m_facts[m_factCount++];
            f.m_lineCount = 0;
            f.m_lines = m_lines + line;
            do
            {
                f.m_lines[f.m_lineCount++] = p; p++;
                while (p < dataEnd && *p != '\r' && *p != '\n') p++;
                *p = 0; p++;
                if (*p == '\r' || *p == '\n')
                {
                    *p = 0;
                    p++;
                }
            } while (p < dataEnd && *p != '\r' && *p != '\n');
            *p = 0; p++;
            if (*p == '\r' || *p == '\n')
            {
                *p = 0;
                p++;
            }
            line += f.m_lineCount;
        }

        m_factCount--;

        return true;
    }

    Fact Facts::GetFact(size_t index) const
    {
        ROB_ASSERT(index < m_factCount);
        return m_facts[index];
    }

    size_t Facts::GetFactCount() const
    { return m_factCount; }

} // sneaky
