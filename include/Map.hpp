
/**THIS FILE CONTAINS A 2D MAP CLASS THAT CAN BE EASILY VIEWED BY HUMANS**/
//written by Leon Frickensmith and Max Archer
//leonfrickensmith@gmail.com

#ifndef MAP_HPP
#define MAP_HPP

#include <iostream>//cout
#include <fstream>//filestream
#include <stdlib.h>//abs

static const float startValue = -9999.0f;//start/default value of map pieces

template <typename T>
struct HeightData
{
    HeightData(T defaultValue = startValue) :
        value(defaultValue)
    {
    }

    //int reliability;//how reliable is this data
    T value;
};

template <typename T>
class Map
{
public:
    Map(const Vec2i& rHalfSize) : defaultValue(startValue)
    {
        nullRep = '-';
        minValue = 0;
        maxValue = 9;

        m_halfSize = rHalfSize;
        m_origin.x = rHalfSize.x+1;
        m_origin.y = rHalfSize.y+1;

        m_cells.resize(2*rHalfSize.x+1);//make x dimension
        for(typename std::vector<std::vector<HeightData<T> > >::iterator it = m_cells.begin(); it != m_cells.end(); ++it)//make y dimension
            it->resize(2*rHalfSize.y+1);
    }


    /**======================**/
    /**Returns a point in the map using standard 2D cartesian coordinates**/
    /**======================**/
    HeightData<T>& getPoint(const Vec2i& rCoord)//returns reference to cell
    {
        if((abs(rCoord.x) < m_halfSize.x) && (abs(rCoord.y) < m_halfSize.y))
            return m_cells[m_origin.x+rCoord.x][(m_origin.y+rCoord.y)];
        else
            return m_garbage;
    }


    /**======================**/
    /**Uses the input map to produce a gradient of this map**/
    /**======================**/
    void makeGradient(Map& rMap)//takes a map and gives it the gradient data
    {
        const float tolerance = 0.5f;

        for(int y=-m_halfSize.x; y!=m_halfSize.y; ++y)
        {
            for(int x=-m_halfSize.x; x!=m_halfSize.x; ++x)
            {
                if(getPoint(Vec2i(x,y)).value != defaultValue)
                    rMap.getPoint(Vec2i(x,y)).value = f_isSteep(getPoint(Vec2i(x,y)).value, getPoint(Vec2i(x,y+1)).value, getPoint(Vec2i(x-1,y)).value, getPoint(Vec2i(x,y-1)).value, getPoint(Vec2i(x+1,y)).value, tolerance);
                else
                    rMap.getPoint(Vec2i(x,y)).value = defaultValue;
            }
        }
    }



    /**======================**/
    /**Takes the values in the map and fits them between max and min**/
    /**======================**/
    void normalizeMap()//take all the cells and put them to values between 0 and 9 (used for human viewing)
    {
        const int width = m_halfSize.x*2+1;
        const int length = m_halfSize.y*2+1;

        for(int y = 0; y<length; ++y)
        {
            for(int x = 0; x<width; ++x)
            {
                if(m_cells[x][y].value != defaultValue)
                    if(m_cells[x][y].value > maxValue)
                        m_cells[x][y].value = maxValue;
            }
        }
        for(int y = 0; y<length; ++y)
        {
            for(int x = 0; x<width; ++x)
            {
                if(m_cells[x][y].value != defaultValue)
                    if(m_cells[x][y].value < minValue)
                        m_cells[x][y].value = minValue;
            }
        }
    }


    /**======================**/
    /**THIS FUNCTION PRINTS THE MAP TO A FILE**/
    /** it makes sure it doesn't print anything below a min value, **/
    /**======================**/
    void toFile(const std::string& rFileName)
    {
        const int width = m_halfSize.x*2+1;
        const int length = m_halfSize.y*2+1;

        std::ofstream file(rFileName.c_str());
        if (file.is_open())
        {
            for(int y = 0; y<length; ++y)
            {
                for(int x = 0; x<width; ++x)
                {
                    if(m_cells[x][y].value == defaultValue)
                    {
                        file << nullRep;//we write two of each because that is aproximately a square in a text file, otherwise it gets skewed
                        file << nullRep;
                    }
                    else
                    {
                        file << static_cast<int>(m_cells[x][y].value);
                        file << static_cast<int>(m_cells[x][y].value);
                    }
                }
                file << "\n";
            }
            file.close();
        }
        else
            std::cout << "\nMap Write Failed";
    }




    char nullRep;//used by print to file to represent no data
    T minValue;
    T maxValue;
    const T defaultValue;
private:


    /**======================**/
    /**TELLS US THE STEEPNESS FACTOR OF A PIECE OF TERRAIN BY COMPARING THE HEIGHT OF ADJACENT PIECES**/
    /**======================**/
    int f_isSteep(T origin, T up, T left, T down, T right, T tolerance)//written by Max Archer 9/17/2014
    {
        if(origin!=defaultValue)
        {
            if(up==defaultValue)
                up = origin;
            if(left==defaultValue)
                left = origin;
            if(down==defaultValue)
                down = origin;
            if(right==defaultValue)
                right = origin;

            if(up-origin >= tolerance or origin-up >= tolerance)
                return 1;
            else if(left-origin >= tolerance or origin-left >= tolerance)
                return 1;
            else if(down-origin >= tolerance or origin-down >= tolerance)
                return 1;
            else if(right-origin >= tolerance or origin-right >= tolerance)
                return 1;
        }
        return 0;
    }

    HeightData<T> m_garbage;
    Vec2i m_halfSize;
    Vec2i m_origin;
    std::vector<std::vector<HeightData<T> > > m_cells;
};



#endif // MAP_HPP
