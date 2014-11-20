
#ifndef H_BACT_HIGH_SCORE_LIST_H
#define H_BACT_HIGH_SCORE_LIST_H

#include "rob/Types.h"

namespace sneaky
{

    class HighScoreList
    {
    public:
        static const size_t MAX_NAME_LENGTH = 16;
        static const size_t MAX_SCORE_COUNT = 10;
        static const size_t INVALID_INDEX = MAX_SCORE_COUNT;
    public:
        HighScoreList();

        bool Load();
        bool Save();

        bool IsHighScore(int score) const;

        /// Adds the new high score and returns it's index.
        size_t AddScore(int score);
        void SetName(size_t index, const char * const name);

        size_t GetIndex(int score) const;

        const char* GetName(size_t index) const;
        int GetScore(size_t index) const;
        size_t GetScoreCount() const;

    private:
        struct Score
        {
            char m_name[MAX_NAME_LENGTH];
            int m_score;
        };
        Score m_scores[MAX_SCORE_COUNT];
        size_t m_scoreCount;
    };

} // sneaky

#endif // H_BACT_HIGHSCORE_LIST_H

