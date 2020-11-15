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

uint32_t process_mark = 0;

/*
* NOTE: This needs a better name. Scheduling Policy sounds so
*       INSURANCEsy
*
*       I believe this is round robin! It jsut looks different thatn what
*		it appears on textsbooks.
*/
tMiniProcess* scheduling_policy_next( tMiniProcess* active_proc, tProcessList* proc_list  ){

	//Increment process mark
	//(skip dead processes)
	uint32_t count=0;
	do{
		count++;
		process_mark = (process_mark + 1) % proc_list->count;
	}while( proc_list->list[process_mark].state == ProcessStateDead );


	//Return process at process mark
	return &(proc_list->list[process_mark]);
}
