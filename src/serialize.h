#ifndef __SERIALIZE_H__
#define __SERIALIZE_H__

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

inline int SerializeWriteSync(int Stream, uint32_t Sync)               { return write(Stream, &Sync, sizeof(uint32_t)); }
inline int SerializeWriteName(int Stream, const char *Name)            { return write(Stream, Name, strlen(Name)+1); }
inline int SerializeWriteData(int Stream, const void *Data, int Bytes) { return write(Stream, Data, Bytes); }
inline int SerializeReadData (int Stream, void *Data, int Bytes)       { return read (Stream, Data, Bytes); }

inline int SerializeWriteSync(FILE *Stream, uint32_t Sync)               { return fwrite(&Sync, 1, sizeof(uint32_t), Stream); }
inline int SerializeWriteName(FILE *Stream, const char *Name)            { return fwrite(Name,  1, strlen(Name)+1,   Stream); }
inline int SerializeWriteData(FILE *Stream, const void *Data, int Bytes) { return fwrite(Data,  1, Bytes,            Stream); }
inline int SerializeReadData (FILE *Stream, void *Data, int Bytes)       { return fread (Data,  1, Bytes,            Stream); }

int SerializeFindSync(int Stream, uint32_t Sync);
int SerializeReadName(int Stream, char *Name, int MaxBytes);

int SerializeFindSync(FILE *Stream, uint32_t Sync);
int SerializeReadName(FILE *Stream, char *Name, int MaxBytes);

#endif // __SERIALIZE_H__
