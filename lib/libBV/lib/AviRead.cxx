//------------------------------------------------------------------------------------------
// (c) Prof. Dr.-Ing. Bernhard Lang
//     HS Osnabrück
//------------------------------------------------------------------------------------------
#include "AviRead.h"
#include "string.h"
//#include <iostream>
//using namespace std;

// #define DEBUG_PRINTOUT

AviRead::AviRead(const char* FileName): FileIn(FileName) {
  char TagBuffer[4];
  long HeaderLength;
  long StreamLength;
  long JunkLength;
  AVI_HDR      avi_hdr;
  AVI_STREAM_HDR stream_hdr;
  VIDS_HDR     vids_hdr;

  error          = 0;
  buffer         = NULL;
  linenumber     = 0;
  framenumber    = 0;
  ErrorBuffer[0] = '\0';

  // RIFF Header
  if (ReadJunkHeader(TagBuffer,FileLength)) {
    sprintf(ErrorBuffer,"cannot read file\n");
    throw this;;
  }
  if (0!=strncmp(TagBuffer,"RIFF",4)) {
    sprintf(ErrorBuffer,"Not a RIFF file\n");
    error=1;
    // // cout << ErrorBuffer << endl;
    throw this;;
  }
  if (ReadTag(TagBuffer)) {
    sprintf(ErrorBuffer,"cannot read file\n");
    throw this;
  }
  if (0!=strncmp(TagBuffer,"AVI ",4)) {
    sprintf(ErrorBuffer,"Not an AVI file\n");
    // cout << ErrorBuffer << endl;
    error=1;
    throw this;
  }

  // cout << "RIFF " << FileLength << "AVI" << endl;

  // RIFF - LIST_hdrl
  // First Element must be a LIST hdrl
  if (ReadJunkHeader(TagBuffer,HeaderLength)) {
    sprintf(ErrorBuffer,"cannot read file\n");
    throw this;
  }
  if (0!=strncmp(TagBuffer,"LIST",4)) {
    sprintf(ErrorBuffer,"Expected 'LIST' for Header but failed\n");
    // cout << ErrorBuffer << endl;
    error=1;
    throw this;
  }
  if (ReadTag(TagBuffer)) {
    sprintf(ErrorBuffer,"cannot read file\n");
    throw this;
  }
  if (0!=strncmp(TagBuffer,"hdrl",4)) {
    sprintf(ErrorBuffer,"Expected Header List 'hdrl' but failed\n");
    // cout << ErrorBuffer << endl;
    error=1;
    throw this;
  }
  // cout << "RIFF - LIST " << FileLength << " hdrl " << endl;

  FileLength -= (8+Align16(HeaderLength)); // Remaining File length
  HeaderLength -= 4;                       // Remove tag length

  // RIFF - LIST_hdrl - avih
  // Expect a "avih"-Junk
  if (ReadJunkHeader(TagBuffer,JunkLength)) {
    sprintf(ErrorBuffer,"cannot read file\n");
    throw this;
  }
  if (0!=strncmp(TagBuffer,"avih",4)) {
    // cout << ErrorBuffer << endl;
    sprintf(ErrorBuffer,"Expected 'avih' Header but failed\n");
    error=1;
    throw this;
  }
  // cout << "RIFF - LIST hdrl - avih " << JunkLength << endl;
  HeaderLength -= (8+Align16(JunkLength));
  if (JunkLength<long(sizeof(AVI_HDR))) {
    sprintf(ErrorBuffer,"Error: AVI-Header to short\n");
    // cout << ErrorBuffer << endl;
    error = 1;
    throw this;
  }
  avi_hdr.us_frame   = get_long();
  avi_hdr.max_bps    = get_long();
  avi_hdr.pad_gran   = get_long();
  avi_hdr.flags     = get_long();
  avi_hdr.tot_frames  = get_long();
  avi_hdr.init_frames = get_long();
  avi_hdr.streams    = get_long();
  avi_hdr.sug_bsize  = get_long();
  avi_hdr.width     = get_long();
  avi_hdr.height    = get_long();
  avi_hdr.scale     = get_long();
  avi_hdr.rate      = get_long();
  avi_hdr.start     = get_long();
  avi_hdr.length    = get_long();
  JunkLength -= 14*4;
  #ifdef DEBUG_PRINTOUT
  printf("  us_frame:   %i\n",avi_hdr.us_frame);
  printf("  max_bps:    %i\n",avi_hdr.max_bps);
  printf("  pad_gran:   %i\n",avi_hdr.pad_gran);
  printf("  flags:    0x%x\n",avi_hdr.flags);
  printf("  tot_frames:  %i\n",avi_hdr.tot_frames);
  printf("  init_frames: %i\n",avi_hdr.init_frames);
  printf("  streams:    %i\n",avi_hdr.streams);
  printf("  sug_bsize:  %i\n",avi_hdr.sug_bsize);
  printf("  width:     %i\n",avi_hdr.width);
  printf("  heigth:    %i\n",avi_hdr.height);
  printf("  scale:     %i\n",avi_hdr.scale);
  printf("  rate:      %i\n",avi_hdr.rate);
  printf("  start:     %i\n",avi_hdr.start);
  printf("  length:    %i\n",avi_hdr.length);
  #endif
  // remove possible unused data at the end of the header
  if (0 < JunkLength) {
    sprintf(ErrorBuffer,"Warning: AVI-Header to long\n");
    // cout << ErrorBuffer << endl;
    if (flush_data(Align16(JunkLength))){
      sprintf(ErrorBuffer,"cannot flush header data\n");
      throw this;
    }
  }

  // Search for Video stream information
  // The for-loop is terminated when the requested video data has been found.
  for (VideoStreamNumber=0; 0<HeaderLength; VideoStreamNumber++) {
    // cout << "HeaderLength:" << HeaderLength << endl;
    // cout << "StreamLength:" << StreamLength << endl;
    // RIFF - LIST_hdrl - LIST_strl
    // Expect another list
    if (ReadJunkHeader(TagBuffer,StreamLength)) {
      sprintf(ErrorBuffer,"cannot read file\n");
      throw this;
    }
    if (0!=strncmp(TagBuffer,"LIST",4)) {
      // remove unexpected element
      sprintf(ErrorBuffer, "Warning: Did find '%c%c%c%c' in Header List\n",
              TagBuffer[0],TagBuffer[1],TagBuffer[2],TagBuffer[3]);
      // cout << ErrorBuffer << endl;
      if (flush_data(Align16(StreamLength))){
        sprintf(ErrorBuffer,"cannot flush data\n");
        throw this;
      }
    }
    if (ReadTag(TagBuffer)) {
      sprintf(ErrorBuffer,"cannot read file\n");
      throw this;
    }
    if (0!=strncmp(TagBuffer,"strl",4)) {
      sprintf(ErrorBuffer, "Error: Did not find Tag 'strl' in Stream Header\n");
      // cout << ErrorBuffer << endl;
      error=1;
      throw this;
    }
    // cout << "RIFF - LIST_hdrl - LIST " << StreamLength << " strl" << endl;
    HeaderLength -= (8+Align16(StreamLength));
    StreamLength -= 4;

    // RIFF - LIST_hdrl - LIST_strl - strh
    if (ReadJunkHeader(TagBuffer,JunkLength)) {
      sprintf(ErrorBuffer,"cannot read file\n");
      throw this;
    }
    if (0!=strncmp(TagBuffer,"strh",4)) {
      sprintf(ErrorBuffer,"Error: Did not find 'strh' Junk\n");
      // cout << ErrorBuffer << endl;
      error=1;
      throw this;
    }
    // cout << "RIFF - LIST_hdrl - LIST strl - strh" << JunkLength << endl;

    StreamLength -= (8+Align16(JunkLength));
    if (ReadTag(stream_hdr.fcc_type)) {
      sprintf(ErrorBuffer,"cannot read file\n");
      throw this;
    }
    stream_hdr.fcc_type[4] = 0;
    if (ReadTag(stream_hdr.fcc_handler)) {
      sprintf(ErrorBuffer,"cannot read file\n");
      throw this;
    }
    stream_hdr.fcc_handler[4] = 0;
    stream_hdr.flags       = get_long();
    stream_hdr.priority    = get_short();
    stream_hdr.language    = get_short();
    stream_hdr.init_frames = get_long();
    stream_hdr.scale       = get_long();
    stream_hdr.rate        = get_long();
    stream_hdr.start       = get_long();
    stream_hdr.length      = get_long();
    stream_hdr.sug_bsize   = get_long();
    stream_hdr.quality     = get_long();
    stream_hdr.samp_size   = get_long();
    JunkLength -= (4*11+2*2);
    switch (JunkLength) {
      case 8:
       stream_hdr.rect_left   = get_short();
       stream_hdr.rect_top    = get_short();
       stream_hdr.rect_right  = get_short();
       stream_hdr.rect_bottom = get_short();
       JunkLength = 0;
       break;
      case 16:
       stream_hdr.rect_left   = get_long();
       stream_hdr.rect_top    = get_long();
       stream_hdr.rect_right  = get_long();
       stream_hdr.rect_bottom = get_long();
       JunkLength = 0;
       break;
    }
    #ifdef DEBUG_PRINTOUT
    printf("  fcc_type:    %s\n",stream_hdr.fcc_type);
    printf("  fcc_handler: %s\n",stream_hdr.fcc_handler);
    printf("  flags:       %i\n",stream_hdr.flags);
    printf("  priority:    %i\n",stream_hdr.priority);
    printf("  language:    %i\n",stream_hdr.language);
    printf("  init_frames: %i\n",stream_hdr.init_frames);
    printf("  scale:       %i\n",stream_hdr.scale);
    printf("  rate:        %i\n",stream_hdr.rate);
    printf("  start:       %i\n",stream_hdr.start);
    printf("  length:      %i\n",stream_hdr.length);
    printf("  sug_bsize:   %i\n",stream_hdr.sug_bsize);
    printf("  quality:     %i\n",stream_hdr.quality);
    printf("  samp_size:   %i\n",stream_hdr.samp_size);
    printf("  rect_left:   %i\n",stream_hdr.rect_left);
    printf("  rect_top:    %i\n",stream_hdr.rect_top);
    printf("  rect_right:  %i\n",stream_hdr.rect_right);
    printf("  rect_bottom: %i\n",stream_hdr.rect_bottom);
    #endif

    // remove possible unused data at the end of the junk
    if (0 < JunkLength) {
      sprintf(ErrorBuffer,"Warning: Stream Header to long\n");
      // cout << ErrorBuffer << endl;
      if (flush_data(Align16(JunkLength))){
        sprintf(ErrorBuffer,"cannot flush data\n");
        throw this;
      }
    }

    if (0==strncmp(stream_hdr.fcc_type,"vids",4)) {
      // got the video header, now read the format data
      if (1) {
        if (0!=strncmp(stream_hdr.fcc_handler,"DIB ",4)) {
         sprintf(ErrorBuffer,"Warnung: DIB-Kennung nicht vorhanden\n");
         // cout << ErrorBuffer << endl;
         error=1;
        }
        // RIFF - LIST_hdrl - LIST_strl - strf
        if (ReadJunkHeader(TagBuffer,JunkLength)) {
          sprintf(ErrorBuffer,"cannot read file\n");
          throw this;
        }
        if (0!=strncmp(TagBuffer,"strf",4)) {
          sprintf(ErrorBuffer,"Error: Did not find 'strf' Junk\n");
          error=1;
          throw this;
        }
        // cout << "RIFF - LIST_hdrl - LIST_strl - strf " << JunkLength << endl;

        StreamLength -= (8+Align16(JunkLength));
        vids_hdr.size      = get_long();
        vids_hdr.width     = get_long();
        vids_hdr.height    = get_long();
        vids_hdr.planes    = get_short();
        vids_hdr.bit_cnt    = get_short();
        vids_hdr.compression = get_long();
        vids_hdr.image_size  = get_long();
        vids_hdr.xpels_meter = get_long();
        vids_hdr.ypels_meter = get_long();
        vids_hdr.num_colors  = get_long();
        vids_hdr.imp_colors  = get_long();
        JunkLength -= (9*4+2*2);
        #ifdef DEBUG_PRINTOUT
        printf("  size:      %i\n",vids_hdr.size);
        printf("  width:     %i\n",vids_hdr.width);
        printf("  height:    %i\n",vids_hdr.height);
        printf("  planes:    %i\n",vids_hdr.planes);
        printf("  bit_cnt:    %i\n",vids_hdr.bit_cnt);
        printf("  compression: %i\n",vids_hdr.compression);
        printf("  image_size:  %i\n",vids_hdr.image_size);
        printf("  xpels_meter: %i\n",vids_hdr.xpels_meter);
        printf("  ypels_meter: %i\n",vids_hdr.ypels_meter);
        printf("  num_colors:  %i\n",vids_hdr.num_colors);
        printf("  imp_colors:  %i\n",vids_hdr.imp_colors);
        #endif
        // read color table
        for (unsigned int i=0;i<vids_hdr.num_colors;i++) {
          if (i<4) {
           printf("Error: not enough LUT-Data\n");
          }
          vids_lut[i][2] = get_char(); // blue
          vids_lut[i][1] = get_char(); // green
          vids_lut[i][0] = get_char(); // red
          get_char();
          JunkLength -= 4;
        }
        // remove possible unused data at the end of the header
        if (0 < JunkLength) {
          sprintf(ErrorBuffer,"Warning: Stream Header to long\n");
          // cout << ErrorBuffer << endl;
          if (flush_data(Align16(JunkLength))){
            sprintf(ErrorBuffer,"cannot flush data\n");
            throw this;
          }
        }
        // remove possible unused data at the end of the list
        if (0 < StreamLength) {
          if (flush_data(Align16(StreamLength))){
            sprintf(ErrorBuffer,"cannot flush data\n");
            throw this;
          }
        }
        // Skip remaining Header data because we are only interested in the
        // Video data
        if (0 < HeaderLength) {
          // cout << "flush " << HeaderLength << endl;
          if (flush_data(Align16(HeaderLength))){
            sprintf(ErrorBuffer,"cannot flush data\n");
            throw this;
          }
        }
        HeaderLength = 0;
        break; // exit "for"-loop
      } else {
        error = 1;
        sprintf(ErrorBuffer,"Error: Only DIB streams supported\n");
        // cout << ErrorBuffer << endl;
        throw this;
      }
    } else {
      // RIFF - LIST_hdrl - LIST_strl - strf
      // Expect another list
      // it is not the video header, thus skip following
      // stream format junk
      if (ReadJunkHeader(TagBuffer,JunkLength)) {
        sprintf(ErrorBuffer,"cannot read file\n");
        throw this;
      }
      if (0!=strncmp(TagBuffer,"strf",4)) {
        sprintf(ErrorBuffer,"Error: Did not find 'strf' Junk\n");
        // cout << ErrorBuffer << endl;
        error=1;
        throw this;
      }
      // cout << "RIFF - LIST_hdrl - LIST_strl - strf " << JunkLength << endl;
      // remove junk data
      if (0 < JunkLength) {
        if (flush_data(Align16(JunkLength))){
          sprintf(ErrorBuffer,"cannot flush data\n");
          throw this;
        }
      }
    }
  }
  if (0 != HeaderLength) {
    sprintf(ErrorBuffer,"Error: Header List inconsistency\n ");
    error=1;
    // cout << ErrorBuffer << endl;
    throw this;
  }
  while (0<FileLength) {
    if (ReadJunkHeader(TagBuffer,MoviLength)) {
      sprintf(ErrorBuffer,"cannot read file\n");
      throw this;
    }
    if (0!=strncmp(TagBuffer,"LIST",4)) {
      // remove junk element
      sprintf(ErrorBuffer, "Info: Remove '%c%c%c%c' Element in AVI file\n",
            TagBuffer[0],TagBuffer[1],TagBuffer[2],TagBuffer[3]);
      //// cout << ErrorBuffer << endl;
      if (flush_data(Align16(MoviLength))){
        sprintf(ErrorBuffer,"cannot flush data\n");
        throw this;
      }
      FileLength -= (8+Align16(MoviLength)); // Remaining File len
    } else {
      if (ReadTag(TagBuffer)) {
        sprintf(ErrorBuffer,"cannot read file\n");
        throw this;
      }
      if (0!=strncmp(TagBuffer,"movi",4)) {
        sprintf(ErrorBuffer, "Warning: Expected 'movi' list but got '%c%c%c%c'\n",
                TagBuffer[0],TagBuffer[1],TagBuffer[2],TagBuffer[3]);
        // cout << ErrorBuffer << endl;
        if (flush_data(Align16(MoviLength-4))){
          sprintf(ErrorBuffer,"cannot flush data\n");
          throw this;
        }
        FileLength -= (8+Align16(MoviLength)); // Remaining len
      } else {
        // Found movi-List
        MoviLength -= 4;  // Set remaining Length of movi-List
        FileLength -= 12; // Set Remaining File len
        // ------------
        break;        // found start of movi-List,
        // ------------  // leave while loop
      }
    }
  }
  width    = avi_hdr.width;          // set width
  height    = avi_hdr.height;         // set height
  framecount = avi_hdr.tot_frames;      // set number of frames
  BitsPerPixel = vids_hdr.bit_cnt;      // set bits per pixel

  BytesPerLine = Align32(width*vids_hdr.bit_cnt/8);
  buffersize = BytesPerLine*height;
  if (buffersize != long(vids_hdr.image_size)) {
    error = 1;
    sprintf(ErrorBuffer,"Error: buffersize != vids_hdr.image_size\n");
    // cout << ErrorBuffer << endl;
    throw this;
  }
  buffer = new unsigned char[buffersize];
}
AviRead::~AviRead(){
  if (NULL!=buffer) {
    delete buffer;
  }
}

int AviRead::flush_data(unsigned int size) {
  unsigned int j;
  char flushbuffer[500];
  for (j=size;j>0;) {
    if (sizeof(flushbuffer)<j) {
      if (sizeof(flushbuffer) != read_buffer(flushbuffer,sizeof(flushbuffer))) {
        sprintf(ErrorBuffer,"Error during flush_data\n");
        // cout << ErrorBuffer << endl;
        error=1;
        return error;
      }
      j -= sizeof(flushbuffer);
    } else {
      if (j != read_buffer(flushbuffer, j)){
        sprintf(ErrorBuffer,"Error during last flush_data\n");
        // cout << ErrorBuffer << endl;
        error = 1;
        return error;
      }
      j=0;
    }
  }
  return 0;
}

int AviRead::ReadJunkHeader(char* buffer, long& size){
  if (4 != read_buffer(buffer, 4)) {
    sprintf(ErrorBuffer,"Cannot read 4 bytes in 'ReadJunkHeader'\n");
    // cout << ErrorBuffer << endl;
    error = 1;
    return error;
  }
  size = get_long();
  return error;
}

int AviRead::ReadTag(char* buffer){
  if (4 != read_buffer(buffer, 4)) {
    sprintf(ErrorBuffer,"Cannot read 4 bytes in 'ReadTag'\n");
    // cout << ErrorBuffer << endl;
    error = 1;
    return error;
  }
  return 0;
}

int AviRead::ReadLine(unsigned char* LineBuffer){
  char  TagBuffer[4];
  long  JunkLength;
  unsigned char* pBuffer;
  if (0==linenumber) {
    if (framenumber==framecount) {
      sprintf(ErrorBuffer, "Error: End of video data\n");
      // cout << ErrorBuffer << endl;
      error=1;
      return error;
    }
    // find stream data
    while (0<=MoviLength) {
      if (ReadJunkHeader(TagBuffer,JunkLength)) {
        return error;
      }
      if (0==strncmp(TagBuffer,"LIST",4)) {
        if (ReadTag(TagBuffer)) {
          return error;
        }
        if (0!=strncmp(TagBuffer,"strl",4)) {
          sprintf(ErrorBuffer, "Error: only 'rec '-Lists allowed in 'movi'-List\n");
          // cout << ErrorBuffer << endl;
          error=1;
          return error;
        }
        MoviLength -= 12;
      } else if ((0==strncmp(TagBuffer+2,"db",2)) || (0==strncmp(TagBuffer+2,"dc",2)))   {
        int ThisStream = ((TagBuffer[0]-'0')<<8) + TagBuffer[1]-'0';
        if (ThisStream != VideoStreamNumber) {
          sprintf(ErrorBuffer, "Error: unexpected video stream\n");
          // cout << ErrorBuffer << endl;
          error = 1;
          return error;
        } else if (JunkLength != buffersize) {
          // remove junk data
          if (0 < JunkLength) {
            if (flush_data(Align16(JunkLength))){
              return error;
            }
          }
          sprintf(ErrorBuffer, "Error: unexpected frame size\n");
          // cout << ErrorBuffer << endl;
          error = 1;
          return error;
        } else {
          if (buffersize != long(read_buffer((char*)buffer, int(JunkLength)))) {
            sprintf(ErrorBuffer, "Error: did read to few video data\n");
            // cout << ErrorBuffer << endl;
            error = 1;
            return error;
          }
        }
        MoviLength -= 8+Align16(JunkLength);
        framenumber++;
        break; // leave "while"-loop
      } else {
        // remove junk data
        if (0 < JunkLength) {
          if (flush_data(Align16(JunkLength))){
            return error;
          }
        }
        MoviLength -= 8+Align16(JunkLength);
      }
    }
  }
  switch (BitsPerPixel) {
    case 24:
     pBuffer = buffer + (BytesPerLine*linenumber);
     for (unsigned i=0;i<width*3;i+=3) {
      LineBuffer[i+0] = *pBuffer++;
      LineBuffer[i+1] = *pBuffer++;
      LineBuffer[i+2]  = *pBuffer++;
     }
     linenumber++;
     if (linenumber >= height) {
       linenumber = 0;
     }
     break;
    default:
     sprintf(ErrorBuffer, "Error: Pixel size %i not yet supported",BitsPerPixel);
     // cout << ErrorBuffer << endl;
     error=1;
     return error;
     break;
  }
  return 0;
}
int AviRead::ReadFrame(unsigned char* buffer) {
  int i;
  for (i=height; 0<i; i--) {
   if (ReadLine(buffer)) return error;
   buffer += 3*width; // get RGB lines
  }
  return 0;
}

AviRead& AviRead::operator>>(Img<RGB_Pixel> &I){
  char  TagBuffer[4];
  long  JunkLength;
  unsigned char* pBuffer;
  if (framenumber==framecount) {
    sprintf(ErrorBuffer, "Error: End of video data\n");
    // cout << ErrorBuffer << endl;
    error=1;
    throw this;
  }
  // find stream data
  while (0<=MoviLength) {
    if (ReadJunkHeader(TagBuffer,JunkLength)) {
      sprintf(ErrorBuffer,"cannot read file\n");
      throw this;
    }
    if (0==strncmp(TagBuffer,"LIST",4)) {
      if (ReadTag(TagBuffer)) {
        sprintf(ErrorBuffer,"cannot read file\n");
        throw this;
      }
      if (0!=strncmp(TagBuffer,"strl",4)) {
        sprintf(ErrorBuffer, "Error: only 'rec '-Lists allowed in 'movi'-List\n");
        // cout << ErrorBuffer << endl;
        error=1;
        throw this;
      }
      MoviLength -= 12;
    } else if ((0==strncmp(TagBuffer+2,"db",2)) || (0==strncmp(TagBuffer+2,"dc",2)))   {
      int ThisStream = ((TagBuffer[0]-'0')<<8) + TagBuffer[1]-'0';
      if (ThisStream != VideoStreamNumber) {
        sprintf(ErrorBuffer, "Error: unexpected video stream\n");
        // cout << ErrorBuffer << endl;
        error = 1;
        throw this;
      } else if (JunkLength != buffersize) {
        // remove junk data
        if (0 < JunkLength) {
          if (flush_data(Align16(JunkLength))){
            sprintf(ErrorBuffer,"cannot flush data\n");
            throw this;
          }
        }
        sprintf(ErrorBuffer, "Error: unexpected frame size\n");
        // cout << ErrorBuffer << endl;
        error = 1;
        throw this;
      } else {
        if (buffersize != long(read_buffer((char*)buffer, int(JunkLength)))) {
          sprintf(ErrorBuffer, "Error: did read to few video data\n");
          // cout << ErrorBuffer << endl;
          error = 1;
          throw this;
        }
      }
      MoviLength -= 8+Align16(JunkLength);
      framenumber++;
      break; // leave "while"-loop
    } else {
      // remove junk data
      if (0 < JunkLength) {
        if (flush_data(Align16(JunkLength))){
          sprintf(ErrorBuffer,"cannot flush data\n");
          throw this;
        }
      }
      MoviLength -= 8+Align16(JunkLength);
    }
  }
  switch (BitsPerPixel) {
    case 24:
     for (unsigned int y=0;y<height;y++) {
      pBuffer = buffer + (BytesPerLine*y);
      for (unsigned int x=0;x<width;x++) {
        I[y][x].Blue(*pBuffer++);
        I[y][x].Green(*pBuffer++);
        I[y][x].Red(*pBuffer++);
      }
     }
     break;
    default:
     sprintf(ErrorBuffer, "Error: Pixel size %i not yet supported",BitsPerPixel);
     // cout << ErrorBuffer << endl;
     error=1;
     throw this;
     break;
  }
  return *this;
}

