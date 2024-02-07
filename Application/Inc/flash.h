/**
 * @file flash.h
 * @author idt12312
 * @date Febrary 7th, 2023
 */

#ifndef FLASH_H_
#define FLASH_H_

#include "main.h"
#include "Maze.h"

#define BACKUP_FLASH_SECTOR_NUM FLASH_SECTOR_1
#define BACKUP_FLASH_SECTOR_SIZE 1024 * 16
#define BACKUP_MAZE_SIZE (MAZE_SIZE * MAZE_SIZE)

bool Flash_clear();
uint8_t *GetWorkRamPointer();
uint8_t *Flash_load();
bool Flash_store();

#endif //  FLASH_H_