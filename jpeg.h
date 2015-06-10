/*
    OGN - Open Glider Network - http://glidernet.org/
    Copyright (c) 2015 The OGN Project

    A detailed list of copyright holders can be found in the file "AUTHORS".

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <stdlib.h>
#include <stdint.h>

#include <setjmp.h>
#include <jpeglib.h>

class JPEG
{ private:
   struct jpeg_compress_struct Compress;           // structures for jpeglib
   struct jpeg_error_mgr       ErrorManager;
   struct jpeg_destination_mgr DestinationManager;
          jmp_buf              ErrorJmp;

  public:
   uint8_t *Data;      // compressed JPEG storage
   size_t   Size;      // actuall JPEG size
   uint32_t Valid;

  private:
   size_t Allocated;   // allocated staorage
   static const size_t AllocUnit = 8192; // storage will be allocated in blocks

  public:
   int Quality;        // JPEG quality: 0..100

  private:
   size_t Reallocate(size_t NewSize)
     { // printf("Reallocate(%d)\n", NewSize);
       Data=(uint8_t *)realloc(Data, NewSize);
       if(Data==0) { Allocated=0; Size=0; return 0; }
       return Allocated=NewSize; }

   static void BufferInit_(jpeg_compress_struct *Compress)
     { JPEG *Client = (JPEG *)Compress->client_data; Client->BufferInit(); }

          void BufferInit(void)
     { if(Allocated<=0) Reallocate(AllocUnit);
       DestinationManager.next_output_byte    = Data;
       DestinationManager.free_in_buffer      = Allocated;
       Size=0; }

   static boolean BufferFull_(jpeg_compress_struct *Compress)
     { JPEG *Client = (JPEG *)Compress->client_data; return Client->BufferFull(); }

          boolean BufferFull(void)
     { Size=Allocated;
       if(Reallocate(Allocated+AllocUnit)<=0)
       { longjmp(ErrorJmp, -1); return FALSE; }
       DestinationManager.next_output_byte    = Data+Size;
       DestinationManager.free_in_buffer      = Allocated-Size;
       return TRUE; }

   static void BufferTerminate_(jpeg_compress_struct *Compress)
     { JPEG *Client = (JPEG *)Compress->client_data; Client->BufferTerminate(); }

          void BufferTerminate(void)
     { Size=Allocated-DestinationManager.free_in_buffer; }

   int Compress_(uint8_t *Image, int Width, int Height,
                 J_COLOR_SPACE ColorSpace, int BytesPerPixel)
     { Compress.image_width      = Width;
       Compress.image_height     = Height;
       Compress.input_components = BytesPerPixel;
       Compress.in_color_space   = ColorSpace;
       jpeg_set_defaults(&Compress);
       jpeg_set_quality(&Compress, Quality, TRUE);

       jpeg_start_compress(&Compress, TRUE);

       int Row=0;
       if(setjmp(ErrorJmp)) { jpeg_abort_compress(&Compress); Size=0; return -1; }

       for(Row=0; Row<Height; Row++, Image+=(Width*BytesPerPixel))
       { jpeg_write_scanlines(&Compress, &Image, 1); }

       jpeg_finish_compress(&Compress);
       return Size; }

  public:
   JPEG()
     { Data=0; Allocated=0; Size=0;
       Quality=80;
       Compress.err = jpeg_std_error(&ErrorManager);
       jpeg_create_compress(&Compress);
       Compress.client_data = this;
       Compress.dest = &DestinationManager;
       DestinationManager.init_destination    = BufferInit_;
       DestinationManager.empty_output_buffer = BufferFull_;
       DestinationManager.term_destination    = BufferTerminate_;
     }

  ~JPEG()
     { jpeg_destroy_compress(&Compress);
       if(Data) free(Data); }

   int Compress_MONO8(uint8_t *Image, int Width, int Height)
     { return Compress_(Image, Width, Height, JCS_GRAYSCALE, 1); }

   int Compress_RGB24(uint8_t *Image, int Width, int Height)
     { return Compress_(Image, Width, Height, JCS_RGB, 3); }

   int Write(char *FileName)
     { if(Data==0) return 0;
       FILE *File = fopen(FileName, "wb"); if(File==0) return 0;
       size_t Written=fwrite(Data, 1, Size, File);
       fclose(File); return Written; }

} ;
