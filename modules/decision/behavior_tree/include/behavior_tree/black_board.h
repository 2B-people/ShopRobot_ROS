#ifndef BLACK_BOARD_H
#define BLACK_BOARD_H

#

class BlackBoard
{
public:
    typedef std::shared_ptr<BlackBoard> Ptr;
    BlackBoard();
    virtual ~BlackBoard() = default;
};


#endif