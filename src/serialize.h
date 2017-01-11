#ifndef __SERIALIZE_H__
#define __SERIALIZE_H__

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

inline int Serialize_WriteSync(int Stream, uint32_t Sync)               { return write(Stream, &Sync, sizeof(uint32_t)); }
inline int Serialize_WriteName(int Stream, const char *Name)            { return write(Stream, Name, strlen(Name)+1); }
inline int Serialize_WriteData(int Stream, const void *Data, int Bytes) { return write(Stream, Data, Bytes); }
inline int Serialize_ReadData (int Stream, void *Data, int Bytes)       { return read (Stream, Data, Bytes); }

inline int Serialize_WriteSync(FILE *Stream, uint32_t Sync)               { return fwrite(&Sync, 1, sizeof(uint32_t), Stream); }
inline int Serialize_WriteName(FILE *Stream, const char *Name)            { return fwrite(Name,  1, strlen(Name)+1,   Stream); }
inline int Serialize_WriteData(FILE *Stream, const void *Data, int Bytes) { return fwrite(Data,  1, Bytes,            Stream); }
inline int Serialize_ReadData (FILE *Stream, void *Data, int Bytes)       { return fread (Data,  1, Bytes,            Stream); }

int Serialize_FindSync(int Stream, uint32_t Sync);
int Serialize_ReadName(int Stream, char *Name, int MaxBytes);

int Serialize_FindSync(FILE *Stream, uint32_t Sync);
int Serialize_ReadName(FILE *Stream, char *Name, int MaxBytes);

#endif // __SERIALIZE_H__
