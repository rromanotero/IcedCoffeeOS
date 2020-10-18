// These are FAT-specific but that's the only one we have

#ifndef FAT_DEF_H
#define FAT_DEF_H

#include <stdbool.h>
#include <stdint.h>

#define FAT_SUCCESS                 0
#define FAT_FILE_NOT_FOUND          1

#define FAT_MAX_FILENAME_LENGTH     8       //Given by FAT format
#define FAT_MAX_EXT_LENGTH          3       //Given by FAT format
#define FAT_MAX_DIR_ENTRIES         500 // TODO this is a system define

typedef struct {
    uint8_t name[FAT_MAX_FILENAME_LENGTH+1];
    uint8_t ext[FAT_MAX_EXT_LENGTH+1];
    uint32_t size;
    uint32_t first_cluster;
    bool is_volume;
    bool is_hidden;
    bool is_system;
    bool is_read_only;
    bool is_archive;
    bool is_directory;
} FATFile;

typedef struct FATDirectory {
    FATFile files[FAT_MAX_DIR_ENTRIES];
    uint32_t num_of_files;
} FATDirectory;

FATDirectory directory_entries(uint8_t *path);
uint32_t open_file(FATFile *file ,char *fileName);
void read_file(FATFile *file, uint8_t *buffer);

#endif
