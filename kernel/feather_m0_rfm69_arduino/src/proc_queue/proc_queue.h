/**
*   This file is part of IcedCoffeeOS
*   (https://github.com/rromanotero/IcedCoffeeOS).
*
*   and adapted from MiniOS:
*   (https://github.com/rromanotero/minios).
*
*   Copyright (c) 2020 Rafael Roman Otero.
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
**/


#ifndef PROC_QUEUE_H_
#define PROC_QUEUE_H_

#define PROC_QUEUE_TOTAL_NUM_OF_QUEUES          PROC_QUEUE_TOTAL_NUM_OF_READY_QUEUES + PROC_QUEUE_TOTAL_NUM_OF_BLOCKED_QUEUES
#define PROC_QUEUE_TOTAL_NUM_OF_READY_QUEUES    5
#define PROC_QUEUE_TOTAL_NUM_OF_BLOCKED_QUEUES  4

typedef enum tProcQueueId {
    ProcQueueReadyRealTime = 0,
    ProcQueueReadySystem,
    ProcQueueReadyUser,
    ProcQueueReadyBatch,
    ProcQueueReadyIdle,
    ProcQueueBlockedRealTime,
    ProcQueueBlockedSystem,
    ProcQueueBlockedUser,
    ProcQueueBlockedBatch
  };



#endif /* PROC_QUEUE_H_ */
