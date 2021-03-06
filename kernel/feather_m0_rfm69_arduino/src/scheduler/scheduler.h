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

#ifndef SCHEDULER_H_
#define SCHEDULER_H_


#define SCHEDULER_PROCESS_CREATE_FAILED		0
#define SCHEDULER_PROCESS_CREATE_SUCCESS	1
#define SCHEDULER_MAX_NUM_PROCESSES			  SYS_SCHED_MAX_NUM_OF_PROCESSES
#define SCHEDULER_THREAD_POOL_SIZE				SCHEDULER_MAX_NUM_PROCESSES

typedef enum tProcessState {
		ProcessStateReady = 0,
		ProcessStateRunning,
		ProcessStateFree,
		ProcessStateNull
	};

typedef struct{
	const char* name;
	uint32_t* sp;
	tProcessState state;
  tProcQueueId queue_id;
}tMiniProcess;


typedef struct{
	tMiniProcess list[SCHEDULER_MAX_NUM_PROCESSES];
	uint32_t count;
}tThreadPool;

#endif /* SCHEDULER_H_ */
