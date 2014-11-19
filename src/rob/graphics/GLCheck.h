
#ifndef H_ROB_GL_CHECK_H
#define H_ROB_GL_CHECK_H

#define GL_CHECK rob::GLCheck(__FILE__, __LINE__)

namespace rob
{

    void GLCheck(const char * const file, const int line);

} // rob

#endif // H_ROB_GL_CHECK_H

