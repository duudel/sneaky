
#ifndef H_ROB_INDEX_BUFFER_H
#define H_ROB_INDEX_BUFFER_H

#include "BufferObject.h"

namespace rob
{

    class IndexBuffer : private BufferObject
    {
    public:
        IndexBuffer();

        using BufferObject::GetObject;
        using BufferObject::GetTarget;

        using BufferObject::Resize;
        using BufferObject::Write;

        using BufferObject::GetSize;
        using BufferObject::IsDynamic;
    };

} // rob

#endif // H_ROB_INDEX_BUFFER_H

