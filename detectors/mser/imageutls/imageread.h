#ifndef __IMAGEREAD_H__
#define __IMAGEREAD_H__

#include <stdio.h>
#include <stdlib.h>

#define NUM_SIGNATURE_BYTES 4

#ifdef WITH_LIBPNG

#  include <png.h>

#  ifndef png_jmpbuf
#     define png_jmpbuf(png_ptr) ((png_ptr)->jmpbuf)
#  endif // png_jmpbuf

#endif // WITH_LIBPNG

#ifdef WITH_LIBJPEG

#ifdef __cplusplus
extern "C" {
#endif

#include <jpeglib.h>

#ifdef __cplusplus
}
#endif

#endif // WITH_LIBJPEG

template<typename PixelType>
int read_image(const char *filename, PixelType *&data,
               size_t &width, size_t &height, size_t &channels)
{
    unsigned char signature_buf[NUM_SIGNATURE_BYTES];
    FILE *f;
    if ((f = fopen(filename, "rb")) == NULL)
        return -1;
    /* Read in some of the signature bytes */
    if (fread(signature_buf, 1,
              NUM_SIGNATURE_BYTES, f) != NUM_SIGNATURE_BYTES)
        return -2;
    /* reset file pointer */
    fseek(f, 0L, SEEK_SET);
    int retval;
    if (signature_buf[0]=='P' &&
            (signature_buf[1]=='5' || signature_buf[1]=='6'))
    {
        retval = read_pnm(f, data, width, height, channels);
    }
    else
#ifdef WITH_LIBJPEG
        if (signature_buf[0]==0xff && signature_buf[1]==0xd8)
        {
            retval = read_jpeg(f, data, width, height, channels);
        }
        else
#endif

#ifdef WITH_LIBPNG
            if (!png_sig_cmp(signature_buf,
                             (png_size_t)0, NUM_SIGNATURE_BYTES))
            {
                retval = read_png(f, data, width, height, channels);
            }
            else
#endif

#ifdef WITH_LIBTIFF
                if ((signature_buf[0]=='I' && signature_buf[1]=='I' &&
                        signature_buf[2]==0x2a && signature_buf[3]==0x00) ||
                        (signature_buf[0]=='I' && signature_buf[1]=='I' &&
                         signature_buf[2]=='N' && signature_buf[3]=='1') ||
                        (signature_buf[0]=='M' && signature_buf[1]=='M' &&
                         signature_buf[2]==0x00 && signature_buf[3]==0x2a))
                    retval = read_tiff(filename, data, width, height, channels);
                else
#endif
                    return -3;
    fclose(f);
    return retval;
}

template<typename PixelType>
int read_pnm(FILE *f, PixelType *&data,
             size_t &width, size_t &height, size_t &channels)
{
    /* check signature and set number of channels accordingly */
    while (fgetc(f) != 'P')
        ;
    if (fgetc(f)=='5')
        channels=1;
    else
        channels=3;
    while (fgetc(f) != '\n')
        ; /* read first line (filetype) */

    while (fscanf(f,"%d %d\n",&width,&height) != 2)
        while (fgetc(f) != '\n')
            ; /* read lines that are comments */

    /* 'width' and 'height' now contain image dimensions */
    while(fgetc(f) != '\n')
        ; /* read line with max-value of pixels */

    data = new PixelType[channels*width*height];
    if (data == NULL)
        return -4;

    /* read image bitmap */
    unsigned char *row_buf;
    size_t size = channels*width;
    PixelType *ptr = data;
    row_buf = (unsigned char*)malloc(size*sizeof(unsigned char));
    if (row_buf == NULL)
    {
        delete [] data;
        return -4;
    }
    for (size_t i=0; i<height; i++)
    {
        if (fread(row_buf, 1, size, f)!=size)
            return -4;
        else
        {
            for (size_t j=0; j<size; j++)
                *ptr++ = (PixelType)row_buf[j];
        }
    }
    free(row_buf);
    return 0;
}

#ifdef WITH_LIBPNG

template<typename PixelType>
int read_png(FILE *f, PixelType *&data,
             size_t &width, size_t &height, size_t &channels)
{
    png_structp png_ptr;
    png_infop info_ptr;
    unsigned int sig_read = 0;

    png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
    if (png_ptr == NULL)
        return -4;

    info_ptr = png_create_info_struct(png_ptr);
    if (info_ptr == NULL)
    {
        png_destroy_read_struct(&png_ptr, NULL, NULL);
        return -4;
    }

    if (setjmp(png_jmpbuf(png_ptr)))
    {
        /* Free all of the memory associated with the png_ptr and info_ptr */
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        /* If we get here, we had a problem reading the file */
        return -4;
    }

    /* Set up the input control if you are using standard C streams */
    png_init_io(png_ptr, f);

    /* If we have already read some of the signature */
    png_set_sig_bytes(png_ptr, sig_read);

    int png_transforms =
        PNG_TRANSFORM_STRIP_16 |
        PNG_TRANSFORM_STRIP_ALPHA |
        PNG_TRANSFORM_PACKING |
        PNG_TRANSFORM_EXPAND;
    /*
     * If you have enough memory to read in the entire image at once,
     * and you need to specify only transforms that can be controlled
     * with one of the PNG_TRANSFORM_* bits (this presently excludes
     * dithering, filling, setting background, and doing gamma
     * adjustment), then you can read the entire image (including
     * pixels) into the info structure with this call:
     */
    png_read_png(png_ptr, info_ptr, png_transforms, NULL);

    width = info_ptr->width;
    height = info_ptr->height;
    channels = info_ptr->channels;
    if (channels!=1 && channels!=3)
        return -4;

    png_bytep *row_pointers;

    row_pointers = png_get_rows(png_ptr, info_ptr);

    unsigned int size = channels*width*height;
    data = new PixelType[size];
    if (data == NULL)
        return -4;

    if (info_ptr->rowbytes > (unsigned int)channels*width)
        return -4;

    /* copy all rows to image buffer */
    PixelType *ptr = data;
    for (size_t i=0; i<height; i++)
        for (size_t j=0; j<info_ptr->rowbytes; j++)
            *ptr++ = row_pointers[i][j];

    /* clean up after the read, and free any memory allocated */
    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
    return 0;
}

#endif // WITH_LIBPNG
#ifdef WITH_LIBJPEG

template<typename PixelType>
int read_jpeg(FILE *f, PixelType *&data,
              size_t &width, size_t &height, size_t &channels)
{
    /* This struct contains the JPEG decompression parameters and pointers to
     * working space (which is allocated as needed by the JPEG library).
     */
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    jmp_buf setjmp_buffer;	/* for return to caller */

    /* More stuff */
    JSAMPARRAY buffer;   /* Output row buffer */
    size_t row_stride;		/* physical row width in output buffer */

    cinfo.err = jpeg_std_error(&jerr);
    /* Establish the setjmp return context for my_error_exit to use. */
    if (setjmp(setjmp_buffer))
    {
        jpeg_destroy_decompress(&cinfo);
        return -4;
    }
    /* Now we can initialize the JPEG decompression object. */
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, f);
    jpeg_read_header(&cinfo, TRUE);

    /* Setup color space and get info from header */
    if (cinfo.jpeg_color_space==JCS_GRAYSCALE)
    {
        cinfo.out_color_space=JCS_GRAYSCALE;
        channels=1;
    }
    else
    {
        channels=3;
        cinfo.out_color_space=JCS_RGB;
    }
    width=cinfo.image_width;
    height=cinfo.image_height;

    /* Start decompressor */
    jpeg_start_decompress(&cinfo);

    /* We may need to do some setup of our own at this point before reading
     * the data.  After jpeg_start_decompress() we have the correct scaled
     * output image dimensions available, as well as the output colormap
     * if we asked for color quantization.
     * In this example, we need to make an output work buffer of the right
     * size.
     */

    row_stride = cinfo.output_width * cinfo.output_components;
    buffer = (*cinfo.mem->alloc_sarray)
             ((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);

    unsigned int size = channels*width*height;
    data = new PixelType[size];
    if (data == NULL)
        return -4;

    if (row_stride > channels*width)
        return -4;

    /* Here we use the library's state variable cinfo.output_scanline as the
     * loop counter, so that we don't have to keep track ourselves.
     */
    PixelType *ptr = data;

    while (cinfo.output_scanline < cinfo.output_height)
    {
        /* jpeg_read_scanlines expects an array of pointers to scanlines.
         * Here the array is only one element long, but you could ask for
         * more than one scanline at a time if that's more convenient.
         */
        jpeg_read_scanlines(&cinfo, buffer, 1);
        /* Assume put_scanline_someplace wants a pointer and sample count. */
        for (size_t j=0; j<row_stride; j++)
            *ptr++ = buffer[0][j];
    }
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    return 0;
}

#endif // WITH_LIBJPEG

#ifdef WITH_LIBTIFF

#include "tiffio.h"

template<typename PixelType>
int read_tiff(const char *filename, PixelType *&data,
              size_t &width, size_t &height, size_t &channels)
{
    TIFF* tif = TIFFOpen(filename, "r");
    if (tif)
    {
        size_t npixels;
        uint32* raster;

        TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
        TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
        channels=3;
        npixels = width * height;
        unsigned int size = channels*width*height;
        data = new PixelType[size];
        if (data ==NULL)
        {
            TIFFClose(tif);
            return -4;
        }
        raster = (uint32*) _TIFFmalloc(npixels * sizeof (uint32));
        PixelType *ptr = data;
        if (raster != NULL)
        {
            if (TIFFReadRGBAImage(tif, width, height, raster, 0))
            {
                for (int i=height-1; i>=0; i--)
                {
                    uint32* tmp = raster+width*i;
                    for (size_t j=0; j<width; j++)
                    {
                        // convert to RGB
                        *ptr++ = (unsigned char)TIFFGetR(*tmp);
                        *ptr++ = (unsigned char)TIFFGetG(*tmp);
                        *ptr++ = (unsigned char)TIFFGetB(*tmp);
                        tmp++;
                    }
                }
            }
            else
            {
                _TIFFfree(raster);
                TIFFClose(tif);
                return -4;
            }
            _TIFFfree(raster);
        }
        else
        {
            TIFFClose(tif);
            return -4;
        }
        TIFFClose(tif);
    }
    else
        return -4;
    return 0;
}
#endif

#endif // __IMAGEREAD_H__

