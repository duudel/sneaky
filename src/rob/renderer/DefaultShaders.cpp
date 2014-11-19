
#define GLSL(x) "#version 120\n" #x

namespace rob
{

    extern const char * const g_colorVertexShader = GLSL(
        uniform mat4 u_projection;
        uniform mat4 u_model;
        uniform vec4 u_position;
        attribute vec2 a_position;
        attribute vec4 a_color;
        varying vec4 v_color;
        void main()
        {
//            vec2 pos = u_position.xy + a_position;
            vec2 pos = a_position;
            gl_Position = u_projection * u_model * vec4(pos, 0.0, 1.0);
            v_color = a_color;
        }
    );

    extern const char * const g_colorFragmentShader = GLSL(
        varying vec4 v_color;
        void main()
        {
            gl_FragColor = v_color;
        }
    );

    extern const char * const g_textureVertexShader = GLSL(
        uniform mat4 u_projection;
        uniform mat4 u_model;
        uniform vec4 u_position;
        attribute vec4 a_position;
        attribute vec4 a_color;
        varying vec2 v_uv;
        varying vec4 v_color;
        void main()
        {
            vec2 pos = a_position.xy;
            gl_Position = u_projection * u_model * vec4(pos, 0.0, 1.0);
            v_uv = a_position.zw;
            v_color = a_color;
        }
    );

    extern const char * const g_textureFragmentShader = GLSL(
        uniform sampler2D u_texture0;
        varying vec2 v_uv;
        varying vec4 v_color;
        void main()
        {
            vec4 color = texture2D(u_texture0, v_uv);
            gl_FragColor = v_color * color;
        }
    );

    extern const char * const g_fontVertexShader = GLSL(
        uniform mat4 u_projection;
        attribute vec4 a_position;
        attribute vec4 a_color;
        varying vec2 v_uv;
        varying vec4 v_color;
        void main()
        {
            gl_Position = u_projection * vec4(a_position.xy * 1.0, 0.0, 1.0);
            v_uv = a_position.zw;
            v_color = a_color;
        }
    );

    extern const char * const g_fontFragmentShader = GLSL(
        uniform sampler2D u_texture0;
        varying vec2 v_uv;
        varying vec4 v_color;
        void main()
        {
            vec4 color = v_color;

//            float alpha = texture2D(u_texture0, v_uv).r;

            float buffer = 0.4975;
            float gamma = 0.014;
//            float buffer = 0.5;
//            float gamma = 0.015;
            float dist = texture2D(u_texture0, v_uv).r;
            float alpha = smoothstep(buffer - gamma, buffer + gamma, dist);

            gl_FragColor = vec4(color.rgb, alpha * color.a);
        }
    );

} // rob

#undef GLSL
