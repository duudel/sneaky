
#ifndef H_SNEAKY_GAME_DATA_H
#define H_SNEAKY_GAME_DATA_H

#include "HighScoreList.h"

namespace sneaky
{

    struct GameData
    {
        int             m_score;
        HighScoreList   m_highScores;
    };

} // sneaky

#endif // H_SNEAKY_GAME_DATA_H

