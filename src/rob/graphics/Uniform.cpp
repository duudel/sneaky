
#include "Uniform.h"

#include <GL/glew.h>

namespace rob
{

    void Uniform_UploadInt(GLint location, const void *data)
    { ::glUniform1i(location, *static_cast<const int*>(data)); }

//    void Uniform_UploadUInt(GLint location, const void *data)
//    { ::glUniform1ui(location, *static_cast<const int*>(data)); }

    void Uniform_UploadFloat(GLint location, const void *data)
    { ::glUniform1f(location, *static_cast<const float*>(data)); }

    void Uniform_UploadVec2(GLint location, const void *data)
    { ::glUniform2fv(location, 1, static_cast<const float*>(data)); }

    void Uniform_UploadVec4(GLint location, const void *data)
    { ::glUniform4fv(location, 1, static_cast<const float*>(data)); }

    void Uniform_UploadMat4(GLint location, const void *data)
    { ::glUniformMatrix4fv(location, 1, GL_TRUE, static_cast<const float*>(data)); }


    //static
    Uniform::UploadFunc Uniform::GetUploadFuncFromType(UniformType type)
    {
        switch (type)
        {
        case UniformType::Int: return Uniform_UploadInt;
//        case UniformType::UInt: return Uniform_UploadUInt;
        case UniformType::Float: return Uniform_UploadFloat;
        case UniformType::Vec2: return Uniform_UploadVec2;
        case UniformType::Vec4: return Uniform_UploadVec4;
        case UniformType::Mat4: return Uniform_UploadMat4;
        }
        return nullptr;
    }

    void Uniform::SetValue(int32_t value)
    {
        m_value.m_int = value;
        m_generation++;
    }

//    void Uniform::SetValue(uint32_t value)
//    {
//        m_value.m_int = value;
//        m_generation++;
//    }

    void Uniform::SetValue(float value)
    {
        m_value.m_float = value;
        m_generation++;
    }

    void Uniform::SetValue(const vec4f &value)
    {
        value.CopyTo(m_value.m_vec4);
        m_generation++;
    }

    void Uniform::SetValue(const vec2f &value)
    {
        value.CopyTo(m_value.m_vec2);
        m_generation++;
    }

    void Uniform::SetValue(const mat4f &value)
    {
        value.CopyTo(m_value.m_mat4);
        m_generation++;
    }

} // rob
