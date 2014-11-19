
#include "GLCheck.h"
#include "../Log.h"

#include <GL/glew.h>

namespace rob
{

    static const char* GLErrorString(GLenum err)
    {
        switch (err)
        {
        case GL_INVALID_ENUM: return "INVALID_ENUM";
        case GL_INVALID_VALUE: return "INVALID_VALUE";
        case GL_INVALID_OPERATION: return "INVALID_OPERATION";
        default:
            return "UNKNOWN_ERROR";
        }
    }

    void GLCheck(const char * const file, const int line)
    {
        GLenum err = ::glGetError();
        while (err != GL_NO_ERROR)
        {
            log::Error("GL error: ", GLErrorString(err), " in ", file, ":", line);
            err = ::glGetError();
        }
    }

} // rob
