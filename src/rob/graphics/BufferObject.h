
#ifndef H_ROB_BUFFER_OBJECT_H
#define H_ROB_BUFFER_OBJECT_H

#include "GLTypes.h"
#include "../Types.h"

namespace rob
{

    class BufferObject
    {
    public:
        explicit BufferObject(GLenum target);
        virtual ~BufferObject();

        GLuint GetObject() const;
        GLenum GetTarget() const;

        /// Resizes this buffer object and sets it dynamic or static.
        /// \pre This buffer must be bind before calling this method.
        void Resize(size_t sizeBytes, bool dynamic);
        /// Writes data to this buffer.
        /// \pre This buffer must be bind before calling this method.
        void Write(size_t offset, size_t size, const void *data);

        size_t GetSize() const;
        bool IsDynamic() const;

    protected:
        GLuint m_object;
        GLenum m_target;
        size_t m_sizeBytes;
        bool m_dynamic;
    };

} // rob

#endif // H_ROB_BUFFER_OBJECT_H

