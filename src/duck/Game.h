
#ifndef H_SNEAKY_GAME_H
#define H_SNEAKY_GAME_H

#include "rob/application/Game.h"
#include "GameData.h"
#include "GameState.h"

namespace sneaky
{

    class Game : public rob::Game
    {
    public:
        bool Initialize() override;
    protected:
        void HandleStateChange(int state) override;
    private:
        GameData m_gameData;
    };

} // sneaky

#endif // H_SNEAKY_GAME_H

