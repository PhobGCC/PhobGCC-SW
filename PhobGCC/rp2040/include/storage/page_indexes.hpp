#ifndef PERSISTENCE__PAGE_INDEXES_HPP
#define PERSISTENCE__PAGE_INDEXES_HPP

//make sure any users are aware of settings that must be written back
#define BATCHSETTINGS

enum class PageIndexes : int {
    MASTER = 0,
    STORAGE = 1
};

#endif
