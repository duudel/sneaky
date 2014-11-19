
#ifndef H_DUCK_PID_CONTROLLER_H
#define H_DUCK_PID_CONTROLLER_H


namespace duck
{

    /*
        Simple PID controller for single float variable
        http://en.wikipedia.org/wiki/PID_controller#Pseudocode
        (Edited style)
    */
    class PidController
    {
    protected:
        float m_gainP;
        float m_gainI;
        float m_gainD;

        float m_currentError;
        float m_previousError;
        float m_integral;
        float m_output;

    public:
        PidController()
            : m_gainP(1.0f)
            , m_gainI(1.0f)
            , m_gainD(1.0f)
            , m_currentError(0.0f)
            , m_previousError(0.0f)
            , m_integral(0.0f)
            , m_output(0.0f)
        { }

        void SetGains(float p, float i, float d) { m_gainP = p; m_gainI = i; m_gainD = d; }

        void SetError( float e ) { m_currentError = e; }

        void Step(float dt)
        {
            m_integral = dt * (m_integral + m_currentError);
            float derivative = (1.0f / dt) * (m_currentError - m_previousError);
            m_output = m_gainP * m_currentError + m_gainI * m_integral + m_gainD * derivative;
            m_previousError = m_currentError;
        }

        float GetOutput() { return m_output; }
    };

} // duck

#endif // H_DUCK_PID_CONTROLLER_H
