#ifndef CONSTS_H
#define CONSTS_H

//define logger macro
#define ERROR(msg) emit log(msg, Consts::LOG_ERROR)
#define WARN(msg) emit log(msg, Consts::LOG_WARNING)
#define STATUS(msg) emit log(msg, Consts::LOG_NORMAL)

namespace Consts{

//constants for logger messages
const int LOG_NORMAL = 0;
const int LOG_ERROR = 1;
const int LOG_APPEND = 2;
const int LOG_WARNING = 3;

//sampling constants
const int PROPOSAL_ADD = 0;
const int PROPOSAL_DEL = 1;
const int PROPOSAL_UPDATE = 2;

}


#endif // CONSTS_H
