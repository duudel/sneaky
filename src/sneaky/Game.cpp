
#include "Game.h"
#include "SneakyState.h"
#include "Facts.h"

#include "rob/application/Window.h"
#include "rob/application/GameState.h"
#include "rob/renderer/TextLayout.h"
#include "rob/input/Keyboard.h"
#include "rob/time/Time.h"
#include "rob/Log.h"

#include <cstdlib>
#include <ctime>

namespace sneaky
{

    using namespace rob;

    class MenuState : public rob::GameState
    {
    public:
        MenuState(GameData &gameData)
            : m_gameData(gameData)
        { }

        bool Initialize() override
        {
            m_gameData.m_score = -1;
            return true;
        }

        ~MenuState()
        { }

        void Render() override
        {
            Delay(20);

            Renderer &renderer = GetRenderer();
            renderer.SetView(GetDefaultView());
            renderer.SetColor(Color(1.0f, 1.0f, 1.0f));
            renderer.BindFontShader();

            const Viewport vp = renderer.GetView().m_viewport;
            TextLayout layout(renderer, vp.w / 2.0f, vp.h / 3.0f);

            renderer.SetFontScale(4.0f);
            layout.AddTextAlignC("Sneaky", 0.0f);
            layout.AddLine();

            renderer.SetFontScale(1.0f);
            layout.AddLine();
            layout.AddLine();

            renderer.SetFontScale(1.0f);
            layout.AddTextAlignR("[space]", -20.0f);
            layout.AddTextAlignL("- New game", 10.0f);
            layout.AddLine();
            layout.AddTextAlignR("[return]", -20.0f);
            layout.AddTextAlignL("- High scores", 10.0f);
            layout.AddLine();
            layout.AddTextAlignR("[esc]", -20.0f);
            layout.AddTextAlignL("- Quit", 10.0f);
            layout.AddLine();
        }

        void OnKeyPress(Keyboard::Key key, Keyboard::Scancode scancode, uint32_t mods) override
        {
            if (key == Keyboard::Key::Escape)
                QuitState();
            if (key == Keyboard::Key::Space)
                ChangeState(STATE_Info);
            if (key == Keyboard::Key::Return)
                ChangeState(STATE_HighScore);
        }
    private:
        GameData &m_gameData;
    };


    class HighScoreState : public rob::GameState
    {
    public:
        HighScoreState(GameData &gameData)
            : m_gameData(gameData)
            , m_scoreIndex(HighScoreList::INVALID_INDEX)
            , m_nameInput()
        {
            m_nameInput.SetLengthLimit(HighScoreList::MAX_NAME_LENGTH - 1);
        }

        bool Initialize() override
        {
            const int score = m_gameData.m_score;
            if (score != -1)
            {
                m_scoreIndex = m_gameData.m_highScores.AddScore(score);
            }
            return true;
        }

        ~HighScoreState()
        { }

        bool InsertingNewScore() const
        { return m_scoreIndex != HighScoreList::INVALID_INDEX; }

        void Render() override
        {
            Delay(20);

            Renderer &renderer = GetRenderer();
            renderer.SetView(GetDefaultView());
            renderer.SetColor(Color(1.0f, 1.0f, 1.0f));
            renderer.BindFontShader();

            const Viewport vp = renderer.GetView().m_viewport;

            TextLayout layout(renderer, vp.w / 2.0f, 10.0f);

            renderer.SetFontScale(3.0f);
            layout.AddTextAlignC("High scores", 0.0f);
            layout.AddLines(2);

            renderer.SetFontScale(1.2f);
            renderer.SetColor(Color(1.0f, 1.0f, 1.0f));
            const HighScoreList &highScores = m_gameData.m_highScores;

            char buf[20];
            for (size_t i = 0; i < highScores.GetScoreCount(); i++)
            {
                StringPrintF(buf, "%i. ", i + 1);
                if (i == m_scoreIndex)
                {
                    renderer.SetColor(Color(0.25f, 1.0f, 0.25f));
                    layout.AddTextAlignL(buf, -260.0f);
                    layout.AddTextInputAlignL(m_nameInput, -200.0f);
                    renderer.SetColor(Color(1.0f, 1.0f, 1.0f));
                }
                else
                {
                    layout.AddTextAlignL(buf, -260.0f);
                    layout.AddTextAlignL(highScores.GetName(i), -200.0f);
                }
                StringPrintF(buf, "%i", highScores.GetScore(i));
                layout.AddTextAlignR(buf, 200.0f);
                layout.AddLine();
            }

            const int emptyLines = HighScoreList::MAX_SCORE_COUNT - highScores.GetScoreCount();
            layout.AddLines(emptyLines);

            renderer.SetFontScale(1.0f);
            layout.AddLine();

            renderer.SetColor(Color(1.0f, 1.0f, 1.0f));
            if (InsertingNewScore())
            {
                layout.AddTextAlignC("Press [return] after entering name", 0.0f);
            }
            else
            {
                layout.AddTextAlignC("Press [return] or [esc] to return to main menu", 0.0f);
            }
        }

        void OnTextInput(const char *str) override
        {
            TextInputHandler(m_nameInput).OnTextInput(str);
        }

        void OnKeyPress(Keyboard::Key key, Keyboard::Scancode scancode, uint32_t mods) override
        {
            if (key == Keyboard::Key::Space && !InsertingNewScore())
            {
                ChangeState(STATE_MainMenu);
            }
            else if (key == Keyboard::Key::Return)
            {
                if (InsertingNewScore())
                {
                    if (m_nameInput.GetLength() > 0)
                    {
                        m_gameData.m_highScores.SetName(m_scoreIndex, m_nameInput.GetText());
                        m_gameData.m_highScores.Save();
                        m_scoreIndex = HighScoreList::INVALID_INDEX;
                    }
                }
                else
                {
                    ChangeState(STATE_MainMenu);
                }
            }
            else if (key == Keyboard::Key::Escape)
            {
                if (InsertingNewScore())
                {
                    if (m_nameInput.GetLength() > 0)
                        m_gameData.m_highScores.SetName(m_scoreIndex, m_nameInput.GetText());
                    else
                        m_gameData.m_highScores.SetName(m_scoreIndex, "<no name>");

                    m_gameData.m_highScores.Save();
                }
                ChangeState(STATE_MainMenu);
            }
        }

        void OnKeyDown(Keyboard::Key key, Keyboard::Scancode scancode, uint32_t mods) override
        {
            TextInputHandler(m_nameInput).OnKeyDown(key, mods);
        }
    private:
        GameData &m_gameData;
        size_t m_scoreIndex;
        TextInput m_nameInput;
    };



    class InfoState : public rob::GameState
    {
    public:
        InfoState()
            : m_facts()
            , m_fact()
            , m_textW(0.0f)
        { }

        bool Initialize() override
        {
            if (!m_facts.Load(GetAllocator()))
            {
                log::Error("Could not load facts");
                return false;
            }
            if (m_facts.GetFactCount() > 0)
            {
                const size_t factIndex = std::rand() % m_facts.GetFactCount();

                m_fact = m_facts.GetFact(factIndex);
                for (size_t line = 0; line + 1 < m_fact.m_lineCount; line++)
                {
                    const char *str = m_fact.m_lines[line];
                    const float lineW = GetRenderer().GetTextWidth(str);
                    m_textW = rob::Max(m_textW, lineW);
                }
                m_textW *= 0.5f;
            }
            return true;
        }

        ~InfoState()
        { }

        void Render() override
        {
            Delay(20);

            Renderer &renderer = GetRenderer();
            renderer.SetView(GetDefaultView());
            renderer.SetColor(Color(1.0f, 1.0f, 1.0f));
            renderer.BindFontShader();

            const Viewport vp = renderer.GetView().m_viewport;
            TextLayout layout(renderer, vp.w / 2.0f, vp.h / 8.0f * 2.0f);

            renderer.SetFontScale(2.0f);
            layout.AddTextAlignC("Brief", 0.0f);
            layout.AddLine();
            renderer.SetFontScale(1.0f);
            layout.AddTextAlignC("There has been an oil accident. Your task is to save the oily birds by cleaning them.", 0.0f);
            layout.AddLine();
            layout.AddTextAlignC("With left mouse button you are able to grab the birds. With the right mouse button", 0.0f);
            layout.AddLine();
            layout.AddTextAlignC("you can wash them, when they are in the water container.", 0.0f);
            layout.AddLines(4);

            renderer.SetFontScale(2.0f);
            layout.AddTextAlignC("Fact", 0.0f);
            layout.AddLines(1);
            renderer.SetFontScale(1.0f);

            size_t line = 0;
            for (; line + 1 < m_fact.m_lineCount; line++)
            {
                const char *str = m_fact.m_lines[line];
                layout.AddTextXAlignL(str, -m_textW);
                layout.AddLine();
            }
            layout.AddLine();
            layout.AddTextAlignR(m_fact.m_lines[line], m_textW);
            layout.AddLine();
    }

    void OnKeyPress(Keyboard::Key key, Keyboard::Scancode scancode, uint32_t mods) override
        {
            if (key == Keyboard::Key::Escape)
                ChangeState(STATE_MainMenu);
            if (key == Keyboard::Key::Space)
                ChangeState(STATE_Game);
        }
    private:
        Facts m_facts;
        Fact m_fact;
        float m_textW;
    };


    bool Game::Initialize()
    {
        std::srand(std::time(0));

        m_window->SetTitle(
        #if defined(ROB_DEBUG)
            "Sneaky - Debug"
        #else
            "Sneaky"
        #endif // ROB_DEBUG
        );

        m_gameData.m_highScores.Load();

        HandleStateChange(STATE_MainMenu);
        return true;
    }

    void Game::HandleStateChange(int state)
    {
        switch (state)
        {
        case STATE_NoChange:    break;

        case STATE_MainMenu:    ChangeState<MenuState>(m_gameData); break;
        case STATE_HighScore:   ChangeState<HighScoreState>(m_gameData); break;
        case STATE_Info:        ChangeState<InfoState>(); break;
        case STATE_Game:        ChangeState<SneakyState>(m_gameData); break;
        default:
            rob::log::Error("Invalid state change (", state, ")");
            break;
        }
    }

} // sneaky
