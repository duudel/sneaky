
#ifndef H_ROB_UNIFORM_H
#define H_ROB_UNIFORM_H

#include "GraphicsTypes.h"
#include "GLTypes.h"
#include "../Types.h"

#include "../math/Types.h"
#include "../math/Vector2.h"
#include "../math/Vector4.h"
#include "../math/Matrix4.h"

namespace rob
{

    struct Uniform
    {
        typedef void (*UploadFunc)(GLint location, const void *data);

        static UploadFunc GetUploadFuncFromType(UniformType type);

        void SetValue(int32_t value);
//        void SetValue(uint32_t value);
        void SetValue(float value);
        void SetValue(const vec2f &value);
        void SetValue(const vec4f &value);
        void SetValue(const mat4f &value);

        static const size_t MAX_NAME_LENGTH = 32;

        UniformType m_type;
        union
        {
            int32_t m_int;
            float m_float;
            float m_vec2[2];
            float m_vec4[4];
            float m_mat4[16];
        } m_value;

        uint32_t    m_generation;
        uint32_t    m_references;
        UploadFunc  m_upload;
        char m_name[MAX_NAME_LENGTH];
    };

} // rob

#endif // H_ROB_UNIFORM_H

