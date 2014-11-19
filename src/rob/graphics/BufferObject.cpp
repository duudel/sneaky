
#include "BufferObject.h"
#include "../Assert.h"

#include "GLCheck.h"
#include <GL/glew.h>

namespace rob
{

    BufferObject::BufferObject(GLenum target)
        : m_object()
        , m_target(target)
        , m_sizeBytes(0)
        , m_dynamic(false)
    {
        ::glGenBuffers(1, &m_object);
        GL_CHECK;
    }

    BufferObject::~BufferObject()
    {
        ::glDeleteBuffers(1, &m_object);
        GL_CHECK;
    }

    GLuint BufferObject::GetObject() const
    { return m_object; }

    GLenum BufferObject::GetTarget() const
    { return m_target; }

    void BufferObject::Resize(size_t sizeBytes, bool dynamic)
    {
        m_sizeBytes = sizeBytes;
        m_dynamic = dynamic;
        ::glBufferData(m_target, sizeBytes, nullptr,
                       dynamic ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);
        GL_CHECK;
    }

    void BufferObject::Write(size_t offset, size_t size, const void *data)
    {
        ROB_ASSERT(offset + size <= m_sizeBytes);
        ::glBufferSubData(m_target, offset, size, data);
        GL_CHECK;
    }

    size_t BufferObject::GetSize() const
    { return m_sizeBytes; }

    bool BufferObject::IsDynamic() const
    { return m_dynamic; }

} // rob
