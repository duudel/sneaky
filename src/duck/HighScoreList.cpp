
#include "HighScoreList.h"

#include "rob/Assert.h"
#include "rob/Log.h"
#include "rob/String.h"

#include <SDL2/SDL.h>
#include <string>
#include <fstream>

namespace sneaky
{

    HighScoreList::HighScoreList()
        : m_scoreCount(0)
    {
        for (size_t i = 0; i < MAX_SCORE_COUNT; i++)
        {
            rob::CopyStringN(m_scores[i].m_name, "");
            m_scores[i].m_score = 0;
        }
    }

    void GetHighScoreFileName(std::string &filename)
    {
//        char * const appPath = ::SDL_GetPrefPath("donkions", "Bacteroids");
//        if (appPath)
//        {
//            filename = appPath;
//            ::SDL_free(appPath);
//        }
//        filename += "highscore.lst";

        filename = "highscore.lst";
    }

    bool HighScoreList::Load()
    {
        std::string filename;
        GetHighScoreFileName(filename);

        std::ifstream in(filename.c_str(), std::ios::binary);
        if (!in.is_open()) return false;

        in.read(reinterpret_cast<char*>(&m_scoreCount), sizeof(uint32_t));
        if (m_scoreCount > MAX_SCORE_COUNT)
        {
            m_scoreCount = 0;
            rob::log::Error("Corrupt high score file: ", filename);
            return false;
        }

        int prevScore = -1;
        for (size_t i = 0; i < m_scoreCount; i++)
        {
            if (!in)
            {
                m_scoreCount = 0;
                rob::log::Error("Corrupt high score file, not enough scores: ", filename);
                return false;
            }
            in.read(reinterpret_cast<char*>(&m_scores[i]), sizeof(Score));

            ROB_WARN(m_scores[i].m_name[MAX_NAME_LENGTH - 1] != 0);
            m_scores[i].m_name[MAX_NAME_LENGTH - 1] = 0;

            if (m_scores[i].m_score < 0)
            {
                m_scoreCount = 0;
                rob::log::Error("Corrupt high score file, negative score: ", filename);
                return false;
            }
            if (prevScore > -1 && m_scores[i].m_score > prevScore)
            {
                m_scoreCount = 0;
                rob::log::Error("Corrupt high score file, scores scrambled: ", filename);
                return false;
            }
            prevScore = m_scores[i].m_score;
        }
        return true;
    }

    bool HighScoreList::Save()
    {
        std::string filename;
        GetHighScoreFileName(filename);

        std::ofstream out(filename.c_str(), std::ios::binary);
        if (!out.is_open()) return false;

        out.write(reinterpret_cast<const char*>(&m_scoreCount), sizeof(uint32_t));
        for (size_t i = 0; i < m_scoreCount; i++)
        {
            out.write(reinterpret_cast<const char*>(&m_scores[i]), sizeof(Score));
        }
        return true;
    }

    size_t HighScoreList::GetIndex(int score) const
    {
        for (size_t i = 0; i < m_scoreCount; i++)
        {
            if (score > m_scores[i].m_score)
                return i;
        }
        return m_scoreCount;
    }

    bool HighScoreList::IsHighScore(int score) const
    { return GetIndex(score) < MAX_SCORE_COUNT; }

    size_t HighScoreList::AddScore(int score)
    {
        const size_t index = GetIndex(score);
        if (index == INVALID_INDEX)
            return index;

        size_t last = MAX_SCORE_COUNT - 1;
        if (m_scoreCount < MAX_SCORE_COUNT)
            last = m_scoreCount++;

        for (size_t i = last; i > index; i--)
            m_scores[i] = m_scores[i - 1];

        rob::CopyStringN(m_scores[index].m_name, "");
        m_scores[index].m_score = score;

        return index;
    }

    void HighScoreList::SetName(size_t index, const char * const name)
    {
        ROB_ASSERT(index < m_scoreCount);
        rob::CopyStringN(m_scores[index].m_name, name);
        m_scores[index].m_name[MAX_NAME_LENGTH - 1] = 0;
    }

    const char* HighScoreList::GetName(size_t index) const
    {
        ROB_ASSERT(index < m_scoreCount);
        return m_scores[index].m_name;
    }

    int HighScoreList::GetScore(size_t index) const
    {
        ROB_ASSERT(index < m_scoreCount);
        return m_scores[index].m_score;
    }

    size_t HighScoreList::GetScoreCount() const
    { return m_scoreCount; }

} // sneaky
