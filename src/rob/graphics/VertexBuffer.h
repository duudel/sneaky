
#ifndef H_ROB_VERTEX_BUFFER_H
#define H_ROB_VERTEX_BUFFER_H

#include "BufferObject.h"

namespace rob
{

    class VertexBuffer : private BufferObject
    {
    public:
        VertexBuffer();

        using BufferObject::GetObject;
        using BufferObject::GetTarget;

        using BufferObject::Resize;
        using BufferObject::Write;

        using BufferObject::GetSize;
        using BufferObject::IsDynamic;
    };

} // rob

#endif // H_ROB_VERTEX_BUFFER_H

