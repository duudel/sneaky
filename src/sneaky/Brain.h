
#ifndef H_SNEAKY_BRAIN_H
#define H_SNEAKY_BRAIN_H

namespace rob
{
    class GameTime;
} // rob

namespace sneaky
{

    class GameObject;

    class Brain
    {
    public:
        Brain()
            : m_owner(nullptr)
        { }

        virtual ~Brain() { }

        virtual void Update(const rob::GameTime &gameTime) = 0;

        void SetOwner(GameObject *owner) { m_owner = owner; }

    protected:
        GameObject *m_owner;
    };

    class Input;

    class PlayerBrain : public Brain
    {
    public:
        PlayerBrain(Input *input)
            : Brain()
            , m_input(input)
        { }
        void Update(const rob::GameTime &gameTime) override;
    private:
        Input *m_input;
    };

} // sneaky

#endif // H_SNEAKY_BRAIN_H

