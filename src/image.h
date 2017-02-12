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
    along with this software.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __IMAGE_H__
#define __IMAGE_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

// #include <png.h>
#include <jpeglib.h>
#include <setjmp.h>

// =============================================================================

template <class Type=float>
 class MonoImage
{ public:
   Type   *Data;        // pointer to image storage
   int     Size;        // actuall (needed) storage size
   int     Allocated;   // allocated storage (when 0 => storage is empty or outside of this object)

   int     Width;       // number of columns
   int     Height;      // number of rows

   MonoImage()
     { Data=0; Width=0; Height=0; Size=0; Allocated=0; }

   MonoImage(int NewWidth, int NewHeight) { New(NewWidth, NewHeight); }

  ~MonoImage()
     { if(Allocated) free(Data); }

   void Free(void)
     { if(Allocated) free(Data);
       Data=0; Size=0; Allocated=0; }

   int Reallocate(int NewSize)
     { // printf("PixelImage::Reallocate(%d) ...\n", NewSize);
       if(Allocated&&(NewSize==Size)) return Size;
       Data=(Type *)realloc(Data, NewSize*sizeof(Type)); if(Data==0) { Allocated=0; Size=0; return 0; }
       Allocated=NewSize; return NewSize; }

   int New(int NewWidth, int NewHeight)
     { // printf("MonoImage::New(%dx%d) ...\n", NewWidth, NewHeight);
       int NewSize=NewWidth*NewHeight;
       if(Allocated<NewSize) Reallocate(NewSize);
       if(Allocated<NewSize) { Width=0; Height=0; Size=0; return 0; }
       Width=NewWidth; Height=NewHeight; Size=NewSize;
       // printf(" Size=%d, Allocated=%d\n", Size, Allocated);
       return Size; }

   template <class NewType>
    int New(MonoImage<NewType> &Ref)
     { return New(Ref.Width, Ref.Height); }

   int NewBlank(int NewWidth, int NewHeight)
      { if(New(NewWidth, NewHeight)<=0) return 0;
        Blank(0); return Size; }

   int setExternal(Type *ExtImage, int ExtWidth, int ExtHeight)
     { if(Allocated) { free(Data); Allocated=0; }
       Data=ExtImage; Width=ExtWidth; Height=ExtHeight;
       return Size=Width*Height; }

// -----------------------------------------------------------------------------

   bool  isPixel(int X, int Y) const         { return (X>=0) && (X<Width) && (Y>=0) && (Y<=Height); }
   void setPixel(int X, int Y, Type Value)   { Data[Y*Width+X]=Value; }
   Type getPixel(int X, int Y) const         { return Data[Y*Width+X]; }
   Type incPixel(int X, int Y, Type Value=1) { return Data[Y*Width+X]+=Value; }

// -----------------------------------------------------------------------------

   template <class ExtType>
    int Copy(ExtType *ExtData, int ExtWidth, int ExtHeight, ExtType Scale=1)
   { if(New(ExtWidth, ExtHeight)<=0) return 0;
     Type *ImgData = Data;
     for(int Row=0; Row<Height; Row++)
     { for(int Col=0; Col<Width; Col++)
       { (*ImgData++) = Scale*(*ExtData++); }
     }
     return Size; }

   template <class ExtType>
    int Copy(MonoImage<ExtType> &ExtImage, ExtType Scale=1)
   { return Copy(ExtImage.Data, ExtImage.Width, ExtImage.Height, Scale); }

   template <class ExtType>
    int Add(ExtType *ExtData, int ExtWidth, int ExtHeight, ExtType Weight=1)
   { if(Size==0) { if(NewBlank(ExtWidth, ExtHeight)<=0) return 0; }
     if( (Width!=ExtWidth) || (Height!=ExtHeight) ) return 0;
     Type *ImgData = Data;
     for(int Row=0; Row<Height; Row++)
     { for(int Col=0; Col<Width; Col++)
       { (*ImgData++) += Weight*(*ExtData++); }
     }
     return Size; }

   template <class ExtType>
    int Add(MonoImage<ExtType> &ExtImage, ExtType Weight=1)
   { return Add(ExtImage.Data, ExtImage.Width, ExtImage.Height, Weight); }

   template <class ExtType>
    int CopyBox(ExtType *ExtData, int ExtWidth, int ExtHeight, int FirstCol, int FirstRow, int Cols, int Rows, ExtType Scale=1)
   { if(New(Cols, Rows)<=0) return 0;
     Type *ImgData = Data;
     for(int Row=0; Row<Height; Row++)
     { int ExtRow = FirstRow+Row;
       if( (ExtRow<0) || (ExtRow>=ExtHeight) )
       { for(int Col=0; Col<Width; Col++) (*ImgData++)=0; continue; }
       ExtType *ExtRowData = ExtData + (ExtRow*ExtWidth);
       int ExtCol = FirstCol;
       for( int Col=0; Col<Width; Col++, ExtCol++)
         { ExtType Data=0;
           if((ExtCol>=0) && (ExtCol<ExtWidth))
             Data=ExtRowData[ExtCol];
           (*ImgData++)=Scale*Data; }
     }
     return Size; }

   template <class ExtType>
    int CopyBox(MonoImage<ExtType> &ExtImage, int FirstCol, int FirstRow, int Cols, int Rows, ExtType Scale=1)
   { return CopyBox(ExtImage.Data, ExtImage.Width, ExtImage.Height, FirstCol, FirstRow, Cols, Rows, Scale); }

   void Blank(Type Level=0)
      { if(Data==0) return;
        Type *Image=Data; int Count=Size; for( ; Count; Count--) (*Image++)=Level; }

// -----------------------------------------------------------------------------

   int WritePGM_8bpp(const char *FileName, Type LogRef=0, Type Scale=1, Type Bias=0) const
     { FILE *File=fopen(FileName,"wb"); if(File==0) return -1;
       fprintf(File,"P5\n%d %d 255\n", Width, Height);
       uint8_t Line[Width];
       Type *Img = Data;
       for(int Row=0; Row<Height; Row++)
       { for(int Col=0; Col<Width; Col++)
         { Type Pixel=(*Img++);
           if(LogRef)
           { if(Pixel) { Pixel=logf((float)Pixel/LogRef); Pixel = Pixel*Scale + Bias; }
             else Pixel=0; }
           else
           { Pixel = Pixel*Scale + Bias; }
           if(Pixel<0x00) Pixel=0x00;
           else if(Pixel>0xFF) Pixel=0xFF;
           Line[Col]=(uint8_t)Pixel; }
         if(fwrite(Line, Width, 1, File)!=1) { fclose(File); return -1; }
       }
       fclose(File);
       return Width*Height; }
/*
   int WritePNG_8bpp(const char *FileName, Type LogRef=0, Type Scale=1, Type Bias=0) const
     { FILE *File=fopen(FileName, "wb"); if(File==0) return -1;

       png_structp Struct;
       png_infop   Info;
       Struct = png_create_write_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
       if(Struct==0) { fclose(File); return -1; }
       Info = png_create_info_struct(Struct);
       if(Info==0) { png_destroy_write_struct(&Struct, (png_infopp)0); fclose(File); return -1; }
       if(setjmp(png_jmpbuf(Struct))) { png_destroy_write_struct(&Struct, (png_infopp)0); fclose(File); return -1; }
       png_set_IHDR(Struct, Info, Width, Height,
                    8, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
                    PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
       png_init_io(Struct, File);
       png_write_info(Struct, Info);

       uint8_t Line[Width];
       for(int Row=0; Row<Height; Row++)
       { Type *Img = Data+(Row*Width);
         for(int Col=0; Col<Width; Col++)
         { Type Pixel=(*Img++);
           if(LogRef)
           { if(Pixel) { Pixel=logf((float)Pixel/LogRef); Pixel = Pixel*Scale + Bias; }
             else Pixel=0; }
           else
           { Pixel = Pixel*Scale + Bias; }
           if(Pixel<0x00) Pixel=0x00;
           else if(Pixel>0xFF) Pixel=0xFF;
           Line[Col]=(uint8_t)Pixel; }
         png_write_row(Struct, Line);
       }

       png_write_end(Struct, Info);
       png_destroy_write_struct(&Struct, (png_infopp)0);
       fclose(File);
       return Width*Height; }
*/

   int WriteJPG_8bpp(int Fd, int Quality=80, Type LogRef=0, Type Scale=1, Type Bias=0) const
     { FILE *File=fdopen(Fd, "wb"); if(File==0) return -1;
       int Size=WriteJPG_8bpp(File, Quality, LogRef, Scale, Bias);
       fclose(File); return Size; }

   int WriteJPG_8bpp(const char *FileName, int Quality=80, Type LogRef=0, Type Scale=1, Type Bias=0) const
     { FILE *File=fopen(FileName, "wb"); if(File==0) return -1;
       int Size=WriteJPG_8bpp(File, Quality, LogRef, Scale, Bias);
       fclose(File); return Size; }

   int WriteJPG_8bpp(FILE *File, int Quality=80, Type LogRef=0, Type Scale=1, Type Bias=0) const
     {
       struct jpeg_compress_struct JpegCompress;
       struct jpeg_error_mgr       JpegErrorManager;
              jmp_buf              JpegErrorJmp;

       JpegCompress.err = jpeg_std_error(&JpegErrorManager);
       jpeg_create_compress(&JpegCompress);
       JpegCompress.image_width  = Width;
       JpegCompress.image_height = Height;
       JpegCompress.input_components = 1;
       JpegCompress.in_color_space = JCS_GRAYSCALE;
       jpeg_set_defaults(&JpegCompress);
       jpeg_set_quality(&JpegCompress, Quality, TRUE);
       jpeg_stdio_dest(&JpegCompress, File);
       jpeg_start_compress(&JpegCompress, TRUE);

       if(setjmp(JpegErrorJmp))
       { jpeg_abort_compress(&JpegCompress);
         jpeg_destroy_compress(&JpegCompress);
         return -1; }

       uint8_t Line[Width];
       for(int Row=0; Row<Height; Row++)     // loop over image lines
       { Type *Img = Data+(Row*Width);
         for(int Col=0; Col<Width; Col++)    // loop over pixels in a line
         { Type Pixel=(*Img++);
           if(LogRef)                        // logarythimc pixel rescale
           { if(Pixel) { Pixel=logf((float)Pixel/LogRef); Pixel = Pixel*Scale + Bias; }
             else Pixel=0; }                 //
           else                              // linear pixel rescale
           { Pixel = Pixel*Scale + Bias; }
           if(Pixel<0x00) Pixel=0x00;        // limit to 8 bits
           else if(Pixel>0xFF) Pixel=0xFF;
           Line[Col]=(uint8_t)Pixel; }
         uint8_t *RowPtr=Line;
         jpeg_write_scanlines(&JpegCompress, &RowPtr, 1);
       }

       jpeg_finish_compress(&JpegCompress);
       jpeg_destroy_compress(&JpegCompress);
       return Width*Height; }

} ;

// =============================================================================

#endif // of __IMAGE_H__

