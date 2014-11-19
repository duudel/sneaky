
#ifndef H_DUCK_FACTS_H
#define H_DUCK_FACTS_H

#include "rob/Types.h"

namespace rob
{
    class LinearAllocator;
} // rob

namespace sneaky
{

    struct Fact
    {
        char **m_lines;
        size_t m_lineCount;
    };

    class Facts
    {
    public:
        static const size_t MAX_FACTS = 50;
        static const size_t MAX_LINES = 500;
    public:
        Facts();
        ~Facts();

        bool Load(rob::LinearAllocator &alloc);
        Fact GetFact(size_t index) const;
        size_t GetFactCount() const;

    private:
        Fact *m_facts;
        size_t m_factCount;

        char **m_lines;
    };

} // sneaky

#endif // H_DUCK_FACTS_H

